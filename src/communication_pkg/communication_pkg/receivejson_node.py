import paho.mqtt.client as mqtt
import json
import os

BROKER = "0.0.0.0"   # ou configurar o IP da VM para 0.0.0.0
PORT = 1883
TOPIC = "pingpong/ros"

OUTPUT_FILE = "recebido.json"

def on_connect(client, userdata, flags, reasonCode, properties=None):
    print("Conectado com código", reasonCode)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    print(f"Mensagem recebida em {msg.topic}")
    try:
        data = json.loads(msg.payload.decode())
        with open(OUTPUT_FILE, "w") as f:
            json.dump(data, f, indent=2)
        print(f"JSON salvo em {OUTPUT_FILE}")
    except Exception as e:
        print(f"Erro ao processar JSON: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_forever()