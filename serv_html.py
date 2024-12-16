from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/joystick', methods=['POST', 'OPTIONS'])
def joystick():
    if request.method == 'OPTIONS':
        print("Received OPTIONS request.")
        return jsonify({"status": "success", "message": "CORS preflight"}), 200

    try:
        data = request.get_json()
        x = data.get('x')
        y = data.get('y')
        print(f"Joystick data received: x={x}, y={y}")
        return jsonify({"status": "success", "message": "Joystick data received"}), 200
    except Exception as e:
        print(f"Error handling joystick data: {e}")
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/command', methods=['POST', 'OPTIONS'])
def command():
    if request.method == 'OPTIONS':
        print("Received OPTIONS request.")
        return jsonify({"status": "success", "message": "CORS preflight"}), 200

    try:
        data = request.get_json()
        command = data.get('command')
        print(f"Command received: {command}")
        return jsonify({"status": "success", "message": f"Command {command} received"}), 200
    except Exception as e:
        print(f"Error handling command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)