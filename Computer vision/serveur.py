from flask import Flask, Response, request, jsonify
import time
import cv2
from queue import Queue
import serial
from collections import deque
from statistics import mode
import logging
import sys

from ai_camera import IMX500Detector

# so we dont see the logs of the camera in the console
logging.getLogger('werkzeug').setLevel(logging.ERROR)

ser = serial.Serial('/dev/ttyACM0',9600,timeout = 1) # depend on the port of the arduino


# Init the camera with the model
model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)
camera.start(show_preview=False)
time.sleep(3)  # To make sure the camera is ready

# center of the frame
center_x=320
center_y=240
w=640
h=480

MAX_BOX = 450 # if the box is bigger than this value, we consider that the person is too close

DELAY = 0.05
HUMAN_SIZE = 1.87
CONVERSION_FACTOR = 0.00024 * 200

MIN_HZ_VALUE = 4.0 # if the distance is less than this value, we consider that the person is horizontaly in front of the camera

CALIBRATED = False

counter_lost = 0 # the counter to know if we lost the person
THRESHOLD_LOST = 40

# return tracking values if errors 
TRACKING_LOST_PERSON = -2
CLIENT_NOT_CONNECTED_ID = -3
client_connected = False


# Configuration Flask
app = Flask(__name__)



frame_queue = Queue(maxsize=10)
receiver_queue = Queue(maxsize=10)

HISTOGRAM_WINDOW_SIZE = 20
tracking_history = deque(maxlen=HISTOGRAM_WINDOW_SIZE)



def ardu_talk(prof, mdist, tracking):
    """ Send the data to the arduino through the serial port.

    Args:
        prof (float): The distance of the person from the camera. (z dimension)
        mdist (float): The distance of the person from the center of the frame. (x dimension)
        tracking (int): The tracking value of the person. (1 if the person is tracked, -1 if not, other values for errors)
    """
    
    try:
        if abs(mdist) < MIN_HZ_VALUE:
            mdist = 0
        
        
        message = f"{prof:.2f},{mdist:.2f},{int(tracking)}\n"

        # Send the message to the Arduino
        ser.write(message.encode('utf-8'))
        print(f"Message sent to the arduino : {message.strip()}")

        # Read the response from the Arduino for debugging
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"Response from the Arduino : {response}")

    except Exception as e:
        print(f"Error sending data to Arduino : {e}")


def smooth_tracking(new_value):
    """ Update the tracking history and return the most frequent value in the window.

    Args:
        new_value (int): The new tracking value to add to the history.

    Returns:
        most_frequent_value (int) : The most frequent value in the tracking history.
    """
    tracking_history.append(new_value)
    
    try:
        most_frequent_value = mode(tracking_history)
    except:
        most_frequent_value = None  # If the history is empty
    
    return most_frequent_value



def calculate_distance(human_size, height_box):
    """ Calculate the distance of the person from the camera. (z dimension)

    Args:
        human_size (float): The size of the person in meters.
        height_box (int): The height of the bounding box of the person in pixels.

    Raises:
        ValueError: if the height of the box is less than or equal to 0.

    Returns:
        distance (float): The distance of the person from the camera.
    """

    if height_box <= 0:
        raise ValueError("The pixel value must be strictly positive.")
    
    
    coeff = -0.02
    ordo = 6.3
    
    # affine function for converting pixels into meters, computed from studies and tests
    if height_box > MAX_BOX:
        distance = 0
    else:
        distance = coeff * (height_box/human_size) + ordo
    
    return distance

# Route to get the full video feed
@app.route('/full_feed')
def video_feed():
    def generate_full_frames():
        global client_connected
        client_connected = True  # The client is connected
        
        try:
            while True:
                frame = camera.picam2.capture_array()

                # Encode the frame to avoid data loss
                _, buffer = cv2.imencode('.jpg', frame)

                # Send the frame
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                time.sleep(DELAY)  # Small delay to avoid overloading the server

        except GeneratorExit:
            print("Client disconnected (full feed).")
        finally:
            client_connected = False

    return Response(generate_full_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# Route to get the flux of the camera with the values
@app.route('/fusion')
def fusion():
    def generate():
        global client_connected
        client_connected = True  # The client is connected
        try:
            while True:
                
                try:
                    frame = camera.picam2.capture_array()
                except Exception as e:
                    print(f"Erreur getting a frame : {e}")
                    continue
            
                # Detect the objects in the frame
                detections = camera.get_detections()
                labels = camera.get_labels()

                # Gather the data of the detected persons
                detection_data = {"detections": []}
                list_persons = []

                if detections is not None:
                    for detection in detections:
                        if int(detection.category) > len(labels) - 1:
                            continue
                        
                        label = labels[int(detection.category)]
                        if label != "person":  # Filter only the persons
                            continue
                        
                        x, y, w, h = detection.box
                        if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                            if y >= 0 and y + h <= frame.shape[0] and x >= 0 and x + w <= frame.shape[1]:
                                cropped_person = frame[y:y+h, x:x+w]
                                
                                # Encoding the cropped to avoid data loss
                                _, buffer = cv2.imencode('.jpg', cropped_person)
                                list_persons.append(buffer.tobytes())
                            else:
                                print(f"Error : cropping region invalid (x: {x}, y: {y}, w: {w}, h: {h}).")
                                continue
                        else:
                            print("Error : the frame is empty or invalid.")
                            continue
                    
                        
                        detection_data['detections'].append({
                            "label": label,
                            "bbox": [x, y, w, h]
                        })
                

                        for cropped in list_persons:
                            detections = detection_data.get("detections", [])
                            for detection in detections:
                                bbox = detection.get("bbox", [])
                                x, y, w, h = bbox

                                val_send = x+w/2 - center_x # distance from the center of the frame

                            # build the message to send to the client
                            yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n' +
                                f'X-bbox: {h}\r\n'.encode('utf-8') +
                                f'X-dist_x: {val_send}\r\n'.encode('utf-8') +
                                b'\r\n' +
                                cropped + b'\r\n')
                else:
                    print("No one on the frame")
                    _, buffer = cv2.imencode('.jpg', frame)
                    image = buffer.tobytes()
                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n' +
                        f'X-bbox: {-1}\r\n'.encode('utf-8') +
                        f'X-dist_x: {-1}\r\n'.encode('utf-8') +
                        b'\r\n' +
                        image + b'\r\n')
                    
                    
                time.sleep(DELAY)
                
        except GeneratorExit:
            client_connected = False
            print("Client disconnected.")
        except Exception as e:
            print(f"Errorr: {e}")
        finally:
            camera.stop()
            print("Camera stopping.")
            ardu_talk(0, 0, CLIENT_NOT_CONNECTED_ID) # We tell the arduino that the client is disconnected
            sys.exit()

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')




@app.route('/update_data', methods=['POST'])
def update_data():
    global client_connected, counter_lost

    try:
        data = request.json
        if 'x_distance' in data and 'height_box' in data and "tracking" in data:
            
            x_distance = float(data['x_distance'])
            height_box = int(data['height_box'])  
            tracking = int(data['tracking'])
            
        

            if client_connected:
                if tracking == 1:
                    counter_lost = 0
                else:
                    counter_lost += 1
                
                if counter_lost > THRESHOLD_LOST:
                    tracking = TRACKING_LOST_PERSON
            else:
                print("Client non connected")
                tracking = CLIENT_NOT_CONNECTED_ID
            
            prof = calculate_distance(HUMAN_SIZE, abs(height_box))
            mdist = CONVERSION_FACTOR * x_distance
            
            track_send = smooth_tracking(tracking)
            ardu_talk(prof, mdist, track_send) 
        
        else:
            print("Could not read anything") 

        return jsonify({"status": "success", "message": "Données reçues avec succès."}), 200
    
    except ValueError:
        return jsonify({"status": "error", "message": "'x_distance' doit être convertible en entier"}), 400
    
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    





if __name__ == "__main__":

    app.run(host='0.0.0.0', port=5000)
