from flask import Flask, request
from datetime import datetime

app = Flask(__name__)

formatted_time = datetime.now().strftime("%Y-%m-%d_%H%M%S")

filepath = "Logs/received_data_" + formatted_time+ ".csv"

with open(filepath, "a") as file:  
    file.write("timestamp,totalStepCount" + "\n")

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.data.decode('utf-8') 

    with open(filepath, "a") as file:  
        file.write(data + "\n")
    
     # Decode the received data
    # For example, store it in a database, write to a file, etc.
    return 'Data received successfully'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)  # Listen on all interfaces on port 8080