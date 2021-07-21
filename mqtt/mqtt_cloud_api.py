import paho.mqtt.client as mqtt
import ssl

host = "mqtt.cloud.pozyxlabs.com"
port = 443
topic = "5f928c413528d3055d3093e5"
username = "5f928c413528d3055d3093e5"
password = "920a09b0-c5c6-4210-9e4c-60a0d0dff352"

def on_connect(client, userdata, flags, rc):
    print(mqtt.connack_string(rc))

# Callback triggered by a new Pozyx data packet
def on_message(client, userdata, msg):
    print("Positioning update:", msg.payload.decode())

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed to topic!")

client = mqtt.Client(transport="websockets")
client.username_pw_set(username, password=password)

# sets the secure context, enabling the WSS protocol
client.tls_set_context(context=ssl.create_default_context())

# set callbacks
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.connect(host, port=port)
client.subscribe(topic)

# works blocking, other, non-blocking, clients are available too.
client.loop_forever()
