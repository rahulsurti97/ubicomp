from flask import Flask, render_template, jsonify, request
import random

app = Flask(__name__)

# Simulated sensor data
sensor_data = []

# Route to fetch sensor data
@app.route('/sensor_data')
def get_sensor_data():
    # Generate random data for demonstration
    global sensor_data
    sensor_data.append(random.randint(1, 100))
    if len(sensor_data) > 10:
        sensor_data = sensor_data[-10:]  # Keep only the last 10 data points
    return jsonify(sensor_data)

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.data.decode('utf-8') 
    print(data)
    # with open(filepath, "a") as file:  
    #     file.write(data + "\n")
    
     # Decode the received data
    # For example, store it in a database, write to a file, etc.
    return jsonify(sensor_data)

# Route to render the UI
@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)