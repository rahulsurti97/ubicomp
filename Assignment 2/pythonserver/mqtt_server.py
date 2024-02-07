import paho.mqtt.client as mqtt

# MQTT broker settings
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_TOPIC = "my_topic"

# Callback function to handle incoming messages
def on_message(client, userdata, message):
    print(f"Received message on topic '{message.topic}': {message.payload.decode()}")

# Create MQTT client instance
client = mqtt.Client()

# Set callback function for incoming messages
client.on_message = on_message

# Connect to MQTT broker
client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)

# Subscribe to MQTT topic
client.subscribe(MQTT_TOPIC)

# Start the MQTT client loop to handle incoming messages
client.loop_forever()
