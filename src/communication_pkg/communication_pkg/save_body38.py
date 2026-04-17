# file: inmoov_arm_mapper_pure_geo_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#! good flex and abdu and elbow flex, not goot rotation

"""
Pure-geometry InMoov arm mapper with decoupled flex/abd and calibrated axial rotation (0..120) from CSV.
- No runtime CSV. Rotation uses constants (scale, offset) derived once from your calibration run.
- Flex/Abd use atan2-based decoupled formulas to avoid bleed.
"""

from __future__ import annotations
import os, json, math
from typing import Dict, Any, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool, Float32

# -------- Mechanical limits (deg) --------
MAX_ABD = 60
MAX_FLEX_SH = 120
MAX_ROT = 120
MAX_ELBOW = 110

# -------- Behavior --------
RATE_HZ = 20.0
SMOOTH_ALPHA = 0.30
SLEW_DEG_PER_SEC = 400.0
ROT_GATE_START = 15.0
ROT_GATE_FULL  = 35.0
MIRROR_ROT_ABS = True
DEADZONE_DEG = 2.0

# -------- Per-side gains (flex/abd) --------
K = {
    "L_ABD": 1.00, "R_ABD": 1.00,
    "L_FLEX": 1.50, "R_FLEX": 1.75,  # slight bump on right flex as requested
    "L_ELB": 1.00, "R_ELB": 1.00,
}

# -------- Axial rotation mapping (from your CSV) --------
# servo = clamp( ROT_SCALE[side] * rot_deg + ROT_OFFSET[side], 0, 120 )
ROT_SCALE  = {"L": 0.9502382602374384, "R": 1.0000000000000000}
ROT_OFFSET = {"L": -49.58937584896781, "R": 0.0}

# -------- Signs & Offsets (mechanical) --------
SIGN = {
    "L_ABD": +1, "R_ABD": +1,
    "L_FLEX": +1, "R_FLEX": +1,
    "L_ROT": +1, "R_ROT": +1,
    "L_ELB": +1, "R_ELB": +1,
}
OFFSET = {
    "L_ABD": 0, "R_ABD": 0,
    "L_FLEX": 0, "R_FLEX": 0,
    "L_ROT": 0, "R_ROT": 0,
    "L_ELB": 0, "R_ELB": 0,
}

JSON_CANDIDATES = [
    "/home/atena/fei-atena-tcc/recebido.json",
    "./recebido.json",
]

# -------- Math helpers --------
def v_norm(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n >= 1e-9 else np.array([0.0, 0.0, 0.0])

def project_on_plane(v: np.ndarray, n: np.ndarray) -> np.ndarray:
    return v - np.dot(v, n) * n

def angle_deg_between(a: np.ndarray, b: np.ndarray) -> float:
    a = v_norm(a); b = v_norm(b)
    d = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(math.acos(d)))

def signed_angle_deg_around(ref: np.ndarray, vec: np.ndarray, axis: np.ndarray) -> float:
    # why: twist around the long axis; ref/vec projected onto plane ⟂ axis
    refp = v_norm(project_on_plane(ref, axis))
    vecp = v_norm(project_on_plane(vec, axis))
    if np.linalg.norm(refp) < 1e-9 or np.linalg.norm(vecp) < 1e-9:
        return 0.0
    x = float(np.dot(refp, vecp))
    y = float(np.dot(axis, np.cross(refp, vecp)))
    return float(np.degrees(math.atan2(y, x)))

def clamp_int(x: float, lo: int, hi: int) -> int:
    if not np.isfinite(x): x = 0.0
    return int(max(lo, min(hi, round(float(x)))))

def slew_limit(prev: int, target: int, dt: float, max_rate: float) -> int:
    if max_rate <= 0.0:
        return target
    max_step = max_rate * dt
    delta = float(target - prev)
    if abs(delta) <= max_step:
        return target
    return int(round(prev + math.copysign(max_step, delta)))

def deadzone(x: float, z: float) -> float:
    return 0.0 if abs(x) < z else x

# -------- JSON helpers --------
def read_json_any(paths) -> Optional[Dict[str, Any]]:
    for p in paths:
        try:
            if os.path.exists(p) and os.stat(p).st_size > 0:
                with open(p, "r", encoding="utf-8") as f:
                    txt = f.read().strip()
                if txt:
                    return json.loads(txt)
        except Exception:
            pass
    return None

def pick_best_body(cf: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    bl = (cf.get("body", {}) or {}).get("body_list", []) or []
    if not bl:
        return None
    return max(bl, key=lambda b: float(b.get("confidence", 0.0)))

def get_kp(body: Dict[str, Any], name: str) -> Optional[np.ndarray]:
    arr = (body.get("keypoints_3d", {}) or {}).get(name)
    if not arr or len(arr) != 3:
        return None
    return np.array([float(arr[0]), float(arr[1]), float(arr[2])], dtype=float)

# -------- Torso frame --------
def compute_torso_frame(body: Dict[str, Any]) -> Optional[Dict[str, np.ndarray]]:
    pelvis = get_kp(body, "PELVIS")
    neck   = get_kp(body, "NECK")
    l_sh   = get_kp(body, "LEFT_SHOULDER")
    r_sh   = get_kp(body, "RIGHT_SHOULDER")
    if pelvis is None or neck is None or l_sh is None or r_sh is None:
        return None

    U = v_norm(neck - pelvis)
    L_raw = v_norm(l_sh - r_sh)
    L = v_norm(project_on_plane(L_raw, U))   # ⟂U to stabilize planes
    F = v_norm(np.cross(U, L))
    tf = body.get("torso_forward")
    if isinstance(tf, list) and len(tf) == 3 and all(np.isfinite(tf)):
        F = v_norm(0.5 * F + 0.5 * v_norm(np.array([float(tf[0]), float(tf[1]), float(tf[2])], dtype=float)))
    R = -L
    D = -U
    if np.linalg.norm(F) < 1e-6:
        F = np.array([0.0, 0.0, 1.0])
    return {"U": U, "D": D, "L": L, "R": R, "F": F}

# -------- Angles (decoupled flex/abd + axial rot) --------
def compute_arm_angles(side: str, body: Dict[str, Any], frame: Dict[str, np.ndarray]) -> Optional[Dict[str, float]]:
    assert side in ("LEFT", "RIGHT")
    S = get_kp(body, f"{side}_SHOULDER")
    E = get_kp(body, f"{side}_ELBOW")
    W = get_kp(body, f"{side}_WRIST")
    if S is None or E is None or W is None:
        return None

    D, L, R, F = frame["D"], frame["L"], frame["R"], frame["F"]
    u = v_norm(E - S)
    f = v_norm(W - E)
    if np.linalg.norm(u) < 1e-9 or np.linalg.norm(f) < 1e-9:
        return None

    # Decoupled flex (atan2 in sagittal): flex=atan2(|F·u|, sqrt((D·u)^2 + (Lat·u)^2))
    lat_axis = L if side == "LEFT" else R
    d   = float(np.dot(D,   u))
    lat = float(np.dot(lat_axis, u))
    fwd = float(np.dot(F,   u))
    denom_flex = math.sqrt(max(0.0, d*d + lat*lat))
    flex = math.degrees(math.atan2(abs(fwd), max(1e-9, denom_flex)))

    # Decoupled abd (atan2 in frontal): abd=atan2(|Lat·u|, sqrt((D·u)^2 + (F·u)^2))
    denom_abd = math.sqrt(max(0.0, d*d + fwd*fwd))
    abd = math.degrees(math.atan2(abs(lat), max(1e-9, denom_abd)))

    flex = deadzone(flex, DEADZONE_DEG)
    abd  = deadzone(abd,  DEADZONE_DEG)

    # Elbow flexion
    sh_to_e = v_norm(S - E)
    elbow = 180.0 - angle_deg_between(sh_to_e, f)

    # Axial rotation: elbow hinge axis vs torso lateral, around u; gate by elbow
    e_axis = v_norm(np.cross(u, f))
    rot_signed = signed_angle_deg_around(lat_axis, e_axis, u)
    rot = abs(rot_signed) if MIRROR_ROT_ABS else rot_signed
    if ROT_GATE_FULL > ROT_GATE_START:
        a = max(0.0, min(1.0, (elbow - ROT_GATE_START) / (ROT_GATE_FULL - ROT_GATE_START)))
        gate = a*a*(3 - 2*a)
        rot *= gate

    for v in (flex, abd, elbow, rot):
        if not np.isfinite(v):
            return None

    return {"flex": float(flex), "abd": float(abd), "elbow": float(elbow), "rot": float(rot)}

# -------- ROS2 Node --------
class InMoovArmMapperPure(Node):
    def __init__(self):
        super().__init__("inmoov_arm_mapper_pure_geo")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = {
            "L_ABD":  self.create_publisher(Int32, "/left/shoulder_abd",  qos),
            "R_ABD":  self.create_publisher(Int32, "/right/shoulder_abd", qos),
            "L_FLEX": self.create_publisher(Int32, "/left/shoulder_flex", qos),
            "R_FLEX": self.create_publisher(Int32, "/right/shoulder_flex",qos),
            "L_ROT":  self.create_publisher(Int32, "/left/shoulder_rot",  qos),
            "R_ROT":  self.create_publisher(Int32, "/right/shoulder_rot", qos),
            "L_ELB":  self.create_publisher(Int32, "/left/elbow_flex",    qos),
            "R_ELB":  self.create_publisher(Int32, "/right/elbow_flex",   qos),
            "LH":     self.create_publisher(Bool,  "/left/hand_open",     qos),
            "RH":     self.create_publisher(Bool,  "/right/hand_open",    qos),
        }
        self.pub_raw = {
            "L_ABD":  self.create_publisher(Float32, "/left/shoulder_abd_raw",  qos),
            "R_ABD":  self.create_publisher(Float32, "/right/shoulder_abd_raw", qos),
            "L_FLEX": self.create_publisher(Float32, "/left/shoulder_flex_raw", qos),
            "R_FLEX": self.create_publisher(Float32, "/right/shoulder_flex_raw",qos),
            "L_ROT":  self.create_publisher(Float32, "/left/shoulder_rot_raw",  qos),
            "R_ROT":  self.create_publisher(Float32, "/right/shoulder_rot_raw", qos),
            "L_ELB":  self.create_publisher(Float32, "/left/elbow_flex_raw",    qos),
            "R_ELB":  self.create_publisher(Float32, "/right/elbow_flex_raw",   qos),
        }

        self.last = {
            "L_ABD": 0, "R_ABD": 0, "L_FLEX": 0, "R_FLEX": 0,
            "L_ROT": 0, "R_ROT": 0, "L_ELB": 0, "R_ELB": 0,
            "LH": False, "RH": False,
        }

        self.timer = self.create_timer(1.0 / RATE_HZ, self.loop)
        self.get_logger().info("inmoov_arm_mapper_pure_geo @ %.1f Hz (decoupled + axial rotation mapping)" % RATE_HZ)

    def loop(self):
        data = read_json_any(JSON_CANDIDATES)
        if not data or "current_frame" not in data:
            self._publish_last(); return

        cf = data["current_frame"]
        hands = cf.get("hands", {}) or {}
        self.last["LH"] = bool(hands.get("left_hand_open", True))
        self.last["RH"] = bool(hands.get("right_hand_open", True))

        body = pick_best_body(cf)
        if not body or body.get("tracking_state") != "OK":
            self._publish_last(); return

        frame = compute_torso_frame(body)
        if not frame:
            self._publish_last(); return

        L  = compute_arm_angles("LEFT",  body, frame)
        R  = compute_arm_angles("RIGHT", body, frame)

        dt = 1.0 / RATE_HZ

        # L raw debug
        if L:
            self.pub_raw["L_FLEX"].publish(Float32(data=L["flex"]))
            self.pub_raw["L_ABD"].publish (Float32(data=L["abd"]))
            self.pub_raw["L_ROT"].publish (Float32(data=L["rot"]))
            self.pub_raw["L_ELB"].publish (Float32(data=L["elbow"]))
            # map flex/abd/elbow via K gains
            self.last["L_FLEX"] = self._map_scalar("L_FLEX", K["L_FLEX"] * L["flex"], 0, MAX_FLEX_SH, dt)
            self.last["L_ABD"]  = self._map_scalar("L_ABD",  K["L_ABD"]  * L["abd"],  0, MAX_ABD,    dt)
            self.last["L_ELB"]  = self._map_scalar("L_ELB",  K["L_ELB"]  * L["elbow"],0, MAX_ELBOW,  dt)
            # map rotation using linear scale+offset (calibrated)
            l_servo_rot = SIGN["L_ROT"] * (ROT_SCALE["L"] * L["rot"] + ROT_OFFSET["L"]) + OFFSET["L_ROT"]
            self.last["L_ROT"]  = self._map_scalar_direct("L_ROT", l_servo_rot, 0, MAX_ROT, dt)

        # R raw debug
        if R:
            self.pub_raw["R_FLEX"].publish(Float32(data=R["flex"]))
            self.pub_raw["R_ABD"].publish (Float32(data=R["abd"]))
            self.pub_raw["R_ROT"].publish (Float32(data=R["rot"]))
            self.pub_raw["R_ELB"].publish (Float32(data=R["elbow"]))
            self.last["R_FLEX"] = self._map_scalar("R_FLEX", K["R_FLEX"] * R["flex"], 0, MAX_FLEX_SH, dt)
            self.last["R_ABD"]  = self._map_scalar("R_ABD",  K["R_ABD"]  * R["abd"],  0, MAX_ABD,    dt)
            self.last["R_ELB"]  = self._map_scalar("R_ELB",  K["R_ELB"]  * R["elbow"],0, MAX_ELBOW,  dt)
            r_servo_rot = SIGN["R_ROT"] * (ROT_SCALE["R"] * R["rot"] + ROT_OFFSET["R"]) + OFFSET["R_ROT"]
            self.last["R_ROT"]  = self._map_scalar_direct("R_ROT", r_servo_rot, 0, MAX_ROT, dt)

        self._publish_last()

    def _map_scalar(self, tag: str, value_deg: float, lo: int, hi: int, dt: float) -> int:
        # why: per-side gain to hit limits; signs/offsets are mechanical
        val = SIGN[tag] * value_deg + OFFSET[tag]
        val = clamp_int(val, lo, hi)
        smoothed = int(round((1.0 - SMOOTH_ALPHA) * self.last[tag] + SMOOTH_ALPHA * val))
        return slew_limit(self.last[tag], smoothed, dt, SLEW_DEG_PER_SEC)

    def _map_scalar_direct(self, tag: str, value_servo_deg: float, lo: int, hi: int, dt: float) -> int:
        # rotation already scaled to servo space by ROT_SCALE/OFFSET
        val = clamp_int(value_servo_deg, lo, hi)
        smoothed = int(round((1.0 - SMOOTH_ALPHA) * self.last[tag] + SMOOTH_ALPHA * val))
        return slew_limit(self.last[tag], smoothed, dt, SLEW_DEG_PER_SEC)

    def _publish_last(self):
        self.pub["LH"].publish(Bool(data=self.last["LH"]))
        self.pub["RH"].publish(Bool(data=self.last["RH"]))
        self.pub["L_ABD"].publish (Int32(data=self.last["L_ABD"]))
        self.pub["R_ABD"].publish (Int32(data=self.last["R_ABD"]))
        self.pub["L_FLEX"].publish(Int32(data=self.last["L_FLEX"]))
        self.pub["R_FLEX"].publish(Int32(data=self.last["R_FLEX"]))
        self.pub["L_ROT"].publish (Int32(data=self.last["L_ROT"]))
        self.pub["R_ROT"].publish (Int32(data=self.last["R_ROT"]))
        self.pub["L_ELB"].publish (Int32(data=self.last["L_ELB"]))
        self.pub["R_ELB"].publish (Int32(data=self.last["R_ELB"]))

# -------- Entrypoint --------
def main(args=None):
    rclpy.init(args=args)
    node = InMoovArmMapperPure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()