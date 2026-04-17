# file: inmoov_body34_mapper.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, math
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool

# ---------- Input ----------
JSON_IN = "/home/atena/fei-atena-tcc/recebido.json"

# ---------- Mechanical limits ----------
MAX_ABD        = 60     # shoulder_out
MAX_FLEX_SH    = 120    # shoulder_lift
MAX_ROT        = 120    # upper_arm_roll
MAX_ELBOW      = 110    # elbow_flex

# ---------- Dynamics ----------
RATE_HZ            = 20.0
SMOOTH_ALPHA       = 0.30
SLEW_DEG_PER_SEC   = 400.0
DEADZONE_DEG       = 2.0

# ---------- Gains / signs ----------
K = {
    "L_FLEX": 1.60, "R_FLEX": 1.90,   # você disse que o direito precisava mais ganho
    "L_ABD":  1.00, "R_ABD":  1.00,
    "L_ROT":  1.00, "R_ROT":  1.00,
    "L_ELB":  1.00, "R_ELB":  1.00,
}
SIGN = {
    "L_FLEX": +1, "R_FLEX": +1,
    "L_ABD":  +1, "R_ABD":  +1,
    "L_ROT":  +1, "R_ROT":  +1,
    "L_ELB":  +1, "R_ELB":  +1,
}
INVERT_ABD = {"L": True, "R": True}  # 0° em repouso, 60° no máximo
ROT_SCALE  = {"L": 1.00, "R": 1.00}
ROT_OFFSET = {"L": 0.00, "R": 0.00}

# ---------- tf: quat->Euler ----------
try:
    import tf_transformations as tft  # ros-<distro>-tf-transformations
except Exception as e:
    raise RuntimeError(
        "Instale tf_transformations:\n"
        "  sudo apt update && sudo apt install ros-${ROS_DISTRO}-tf-transformations"
    ) from e

EULER_AXES = "sxyz"  # roll(X), pitch(Y), yaw(Z) – casa com eixos do URDF

# ---------- Math helpers ----------
def clamp_i(x: float, lo: int, hi: int) -> int:
    if not isinstance(x, (int,float)) or not math.isfinite(x): x = 0.0
    return int(max(lo, min(hi, round(float(x)))))

def slew(prev: int, target: int, dt: float, rate: float) -> int:
    if rate <= 0: return target
    step = rate * dt
    d = float(target - prev)
    if abs(d) <= step: return target
    return int(round(prev + math.copysign(step, d)))

def dead(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x

def q_from_obj(obj: Dict[str, float]) -> Optional[Tuple[float,float,float,float]]:
    try: return (float(obj["ox"]), float(obj["oy"]), float(obj["oz"]), float(obj["ow"]))
    except Exception: return None

def q_mul(a: Tuple[float,float,float,float], b: Tuple[float,float,float,float]) -> Tuple[float,float,float,float]:
    ax,ay,az,aw = a; bx,by,bz,bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz
    )

def q_conj(q): x,y,z,w = q; return (-x,-y,-z,w)

def q_rotate_vec(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    return q_mul(q_mul(q, qv), q_conj(q))[:3]

def q_angle_deg(q) -> float:
    w = max(-1.0, min(1.0, q[3]))
    return 2.0 * math.degrees(math.acos(w))

def euler_deg_sxyz(q):
    rx, ry, rz = tft.euler_from_quaternion(q, axes=EULER_AXES)
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))  # roll, pitch, yaw

def swing_twist_deg(q_rel, axis=(0.0,0.0,1.0)) -> float:
    ax, ay, az = axis
    qx, qy, qz, qw = q_rel
    dot = qx*ax + qy*ay + qz*az
    tx, ty, tz = ax*dot, ay*dot, az*dot
    n = math.sqrt(tx*tx + ty*ty + tz*tz + qw*qw)
    if n == 0.0: return 0.0
    tx, ty, tz, tw = tx/n, ty/n, tz/n, qw/n
    ang = 2.0 * math.atan2(math.sqrt(tx*tx + ty*ty + tz*tz), abs(tw)) * (180.0/math.pi)
    # manter módulo; sinal pode ser tratado por SIGN["*_ROT"] se necessário
    return ang

# ---------- Canonical torso axes ----------
DOWN  = (0.0, -1.0, 0.0)
FWD   = (0.0,  0.0, 1.0)
LEFT  = (1.0,  0.0, 0.0)
RIGHT = (-1.0, 0.0, 0.0)

# ---------- Neutral shoulder quats (do seu dump Body34 anterior) ----------
NEUTRAL_SHOULDER_LEFT  = ( 0.0549155287, -0.0286880657,  0.6598029137, 0.7488800883)
NEUTRAL_SHOULDER_RIGHT = (-0.1685325205, -0.0191642605, -0.6405009627, 0.7489913106)

# ---------- JSON helpers ----------
def read_json(path=JSON_IN) -> Optional[Dict[str, Any]]:
    try:
        if not os.path.exists(path) or os.stat(path).st_size == 0: return None
        with open(path, "r", encoding="utf-8") as f:
            s = f.read().strip()
        return json.loads(s) if s else None
    except Exception:
        return None

def pick_best_body(cf: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    bl = (cf.get("body", {}) or {}).get("body_list", []) or []
    if not bl: return None
    return max(bl, key=lambda b: float(b.get("confidence", 0.0)))

# ---------- Angle extraction ----------
def flex_from_euler_pitch(qS: Tuple[float,float,float,float]) -> float:
    roll, pitch, _ = euler_deg_sxyz(qS)
    return abs(dead(pitch, DEADZONE_DEG))

def abd_from_humerus_vector(qS: Tuple[float,float,float,float], side: str) -> float:
    # abdução = inclinação lateral no plano frontal (X vs {Z,Y})
    u = q_rotate_vec(qS, DOWN)
    ux, uy, uz = u
    lat = abs(ux if side=="LEFT" else -ux)  # magnitude lateral
    d_f = math.sqrt(max(0.0, uy*uy + uz*uz))
    return math.degrees(math.atan2(lat, d_f))

def rot_from_twist(qS: Tuple[float,float,float,float], neutral: Tuple[float,float,float,float]) -> float:
    q_rel = q_mul(qS, q_conj(neutral))
    return swing_twist_deg(q_rel, axis=(0.0,0.0,1.0))

def elbow_from_quat(qE: Tuple[float,float,float,float]) -> float:
    return q_angle_deg(qE)

# ---------- ROS2 Node ----------
class InmoovBody34Mapper(Node):
    def __init__(self):
        super().__init__("inmoov_body34_mapper")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
        self.last = {k: 0 for k in ["L_ABD","R_ABD","L_FLEX","R_FLEX","L_ROT","R_ROT","L_ELB","R_ELB"]}
        self.last.update({"LH": False, "RH": False})
        self.create_timer(1.0 / RATE_HZ, self.loop)
        self.get_logger().info("inmoov_body34_mapper: flex=pitch(Y), abd=vetor úmero (plano frontal), rot=twist(Z).")

    def loop(self):
        data = read_json()
        if not data or "current_frame" not in data:
            self._publish(); return

        cf = data["current_frame"]
        hands = cf.get("hands", {}) or {}
        self.last["LH"] = bool(hands.get("left_hand_open",  False))
        self.last["RH"] = bool(hands.get("right_hand_open", False))

        body = pick_best_body(cf)
        if not body:
            self._publish(); return

        qblk = body.get("local_orientation_quat", {}) or {}
        need = ("ShoulderLeft","ShoulderRight","ElbowLeft","ElbowRight")
        if not all(n in qblk for n in need):
            self._publish(); return

        qSL = q_from_obj(qblk["ShoulderLeft"])
        qSR = q_from_obj(qblk["ShoulderRight"])
        qEL = q_from_obj(qblk["ElbowLeft"])
        qER = q_from_obj(qblk["ElbowRight"])
        if any(q is None for q in (qSL,qSR,qEL,qER)):
            self._publish(); return

        # ---- Flex (bom no seu setup): Euler pitch(Y) ----
        flexL = SIGN["L_FLEX"] * (K["L_FLEX"] * flex_from_euler_pitch(qSL))
        flexR = SIGN["R_FLEX"] * (K["R_FLEX"] * flex_from_euler_pitch(qSR))

        # ---- Abdução (decoupled via vetor do úmero; não usa Euler) ----
        abdL  = SIGN["L_ABD"]  * (K["L_ABD"]  * abd_from_humerus_vector(qSL, "LEFT"))
        abdR  = SIGN["R_ABD"]  * (K["R_ABD"]  * abd_from_humerus_vector(qSR, "RIGHT"))
        if INVERT_ABD["L"]: abdL = MAX_ABD - abdL
        if INVERT_ABD["R"]: abdR = MAX_ABD - abdR

        # ---- Rotação axial (twist no eixo Z local relativo ao neutro) ----
        rotL  = SIGN["L_ROT"] * (ROT_SCALE["L"] * (K["L_ROT"] * rot_from_twist(qSL, NEUTRAL_SHOULDER_LEFT))  + ROT_OFFSET["L"])
        rotR  = SIGN["R_ROT"] * (ROT_SCALE["R"] * (K["R_ROT"] * rot_from_twist(qSR, NEUTRAL_SHOULDER_RIGHT)) + ROT_OFFSET["R"])

        # ---- Cotovelo ----
        elbL  = SIGN["L_ELB"] * (K["L_ELB"] * elbow_from_quat(qEL))
        elbR  = SIGN["R_ELB"] * (K["R_ELB"] * elbow_from_quat(qER))

        # ---- Clamp + smooth + slew ----
        dt = 1.0 / RATE_HZ
        self.last["L_FLEX"] = self._shape("L_FLEX", flexL, 0, MAX_FLEX_SH, dt)
        self.last["R_FLEX"] = self._shape("R_FLEX", flexR, 0, MAX_FLEX_SH, dt)
        self.last["L_ABD"]  = self._shape("L_ABD",  abdL,  0, MAX_ABD,    dt)
        self.last["R_ABD"]  = self._shape("R_ABD",  abdR,  0, MAX_ABD,    dt)
        self.last["L_ROT"]  = self._shape("L_ROT",  rotL,  0, MAX_ROT,    dt)
        self.last["R_ROT"]  = self._shape("R_ROT",  rotR,  0, MAX_ROT,    dt)
        self.last["L_ELB"]  = self._shape("L_ELB",  elbL,  0, MAX_ELBOW,  dt)
        self.last["R_ELB"]  = self._shape("R_ELB",  elbR,  0, MAX_ELBOW,  dt)

        self._publish()

    def _shape(self, tag: str, val: float, lo: int, hi: int, dt: float) -> int:
        # comentários só no "porquê": smoothing evita jitter; slew protege os servos
        val = clamp_i(val, lo, hi)
        smooth = int(round((1.0 - SMOOTH_ALPHA) * self.last[tag] + SMOOTH_ALPHA * val))
        return slew(self.last[tag], smooth, dt, SLEW_DEG_PER_SEC)

    def _publish(self):
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

def main(args=None):
    rclpy.init(args=args)
    node = InmoovBody34Mapper()
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