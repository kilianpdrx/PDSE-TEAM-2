from flask import Flask, Response, request, jsonify
import time
import cv2
import uuid
from queue import Queue
import threading
import json
import serial
import math
import sys
from collections import deque
from statistics import mode

from ai_camera import IMX500Detector


# ser = serial.Serial('/dev/ttyAMA10',9600,timeout = 1)
ser = serial.Serial('/dev/ttyACM0',9600,timeout = 1)


# Initialisation du modèle et de la caméra
model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)
# Démarrage de la caméra avec aperçu activé
camera.start(show_preview=False)
time.sleep(3)  # Laisser le temps à la caméra de démarrer

center_x=320
center_y=240
w=640
h=480
MAX_BOX = 450 # normalement 480 mais un peu moins pour avoir de la marge

DELAY = 0.001
HUMAN_SIZE = 1.87
CONVERSION_FACTOR = 0.00024 

MIN_HZ_VALUE = 0.2



counter_lost = 0
THRESHOLD_LOST = 40
TRACKING_LOST_PERSON = -2
CLIENT_NOT_CONNECTED_ID = -3


client_connected = False

IDX = 0 # index du message envoyé

# Configuration Flask
app = Flask(__name__)



frame_queue = Queue(maxsize=10)
receiver_queue = Queue(maxsize=10)

# Associe une URL (/video_feed) à une fonction Python
# Quand un utilisateur accède à http://<ip>:5000/video_feed, la fonction video_feed est exécutée.

HISTOGRAM_WINDOW_SIZE = 20
tracking_history = deque(maxlen=HISTOGRAM_WINDOW_SIZE)



def ardu_talk(prof, mdist, tracking):
    global IDX
    
    
    try:
        # Convertir les données en une chaîne formatée pour l'Arduino
        # message = f"{dist:.2f},{math.degrees(angle):.2f}\n"  # Format : "distance,angle\n"
        # message = f"{IDX}, {prof:.2f},{mdist:.2f},{int(tracking)}\n"  # Format : "distance,angle\n"
        
        if abs(mdist) < MIN_HZ_VALUE:
            mdist = 0
        
        
        message = f"{prof:.2f},{mdist:.2f},{int(tracking)}\n"  # Format : "distance,angle\n"

        IDX += 1
        print("Before writing")
        # Envoyer le message sur le port série
        ser.write(message.encode('utf-8'))
        print("After writing")
        print(f"Message envoyé à l'Arduino : {message.strip()}, TRACKING = {tracking}")

        # Lire la réponse de l'Arduino (facultatif)
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"Réponse de l'Arduino : {response}")

    except Exception as e:
        print(f"Erreur lors de l'envoi des données à l'Arduino : {e}")


def smooth_tracking(new_value):
    """
    Met à jour l'historique des valeurs de tracking et renvoie la valeur la plus fréquente.
    
    :param new_value: La nouvelle valeur de tracking à ajouter.
    :return: La valeur la plus fréquente dans la fenêtre d'historique.
    """
    # Ajouter la nouvelle valeur à l'historique
    tracking_history.append(new_value)
    
    # Calculer la valeur la plus fréquente
    try:
        most_frequent_value = mode(tracking_history)
    except:
        most_frequent_value = None  # Si aucune valeur présente
    
    return most_frequent_value



def calculate_angle(xdist, depth):

    # Vérification des entrées
    if depth == 0:
        raise ValueError("La profondeur ne peut pas être nulle pour éviter une division par zéro.")
    
    # Calcul de l'angle
    mdist = CONVERSION_FACTOR * xdist
    angle = math.atan2(mdist, depth)
    return angle



def calculate_distance(human_size, height_box):

    if height_box <= 0:
        raise ValueError("La valeur en pixels doit être strictement positive.")
    
    
    coeff = -0.02
    ordo = 6.3
    # fonction affine pour la conversion de pixels en mètres
    if height_box > MAX_BOX:
        distance = 0
    else:
        distance = coeff * (height_box/human_size) + ordo
    
    return distance

# Thread pour capturer les frames et leurs données
def capture_frames_and_data():
    while True:
        if not frame_queue.full():
            # Capture d'une image
            frame = camera.picam2.capture_array()
            
            # Détection des objets
            detections = camera.get_detections()
            labels = camera.get_labels()

            # Rassembler les données de détection
            detection_data = {"detections": []}
            list_persons = []

            for detection in detections:
                if int(detection.category) > len(labels) - 1:
                    continue
                
                label = labels[int(detection.category)]
                if label != "person":  # Filtre uniquement pour les personnes
                    continue
                
                x, y, w, h = detection.box
                if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                    if y >= 0 and y + h <= frame.shape[0] and x >= 0 and x + w <= frame.shape[1]:
                        cropped_person = frame[y:y+h, x:x+w]
                        
                        # Encodage du recadré pour éviter la perte d'information
                        _, buffer = cv2.imencode('.jpg', cropped_person)
                        list_persons.append(buffer.tobytes())
                    else:
                        print(f"Erreur : Région de découpage invalide (x: {x}, y: {y}, w: {w}, h: {h}).")
                        continue
                else:
                    print("Erreur : La frame est vide ou invalide.")
                    continue
                # Encodage du recadré pour éviter la perte d'information
                
                detection_data['detections'].append({
                    "label": label,
                    "bbox": [x, y, w, h]
                })

            # Mettre les images recadrées et les données dans la file
            frame_queue.put((list_persons, detection_data))
        else:
            print("Queue is full")
        time.sleep(DELAY)  # Capture ~33 FPS




# Route pour le flux vidéo complet
@app.route('/full_feed')
def video_feed():
    def generate_full_frames():
        global client_connected
        client_connected = True  # Le client est maintenant connecté
        
        try:
            while True:
                frame = camera.picam2.capture_array()

                # Encodage de l'image complète
                _, buffer = cv2.imencode('.jpg', frame)

                # Envoi de l'image
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                time.sleep(DELAY)  # Petite pause pour réduire la charge

        except GeneratorExit:
            print("Client déconnecté (full feed).")
        finally:
            client_connected = False

    return Response(generate_full_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/cropped_feed')
def cropped_feed():
    def generate_cropped_frames():
        global client_connected
        client_connected = True  # Le client est maintenant connecté
        try:
            while True:
                if not frame_queue.empty():
                    cropped_images, _ = frame_queue.get()

                    for cropped in cropped_images:
                        # Générer un identifiant unique
                        image_id = uuid.uuid4().hex

                        # Envoi de l'image avec un identifiant unique
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n' +
                               f'X-Image-ID: {image_id}\r\n\r\n'.encode('utf-8') +
                               cropped + b'\r\n')

                time.sleep(DELAY)  # Petite pause pour réduire la charge

        except GeneratorExit:
            print("Client déconnecté (cropped feed).")
        finally:
            client_connected = False


    return Response(generate_cropped_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# @app.route('/fusion')
# def fusion():
#     def generate():
#         global client_connected
#         client_connected = True  # Le client est maintenant connecté
#         try:
#             while True:
#                 if not frame_queue.empty():
#                     cropped_frames, extra_data = frame_queue.get()  # Récupérer frame et valeur supplémentaire

#                     for cropped in cropped_frames:


#                         # Récupérer la valeur spécifique à envoyer
#                         detections = extra_data.get("detections", [])
#                         for detection in detections:
#                             bbox = detection.get("bbox", [])
#                             x, y, w, h = bbox

#                             val_send = x+w/2 - center_x

#                         # Ajouter la valeur en tant qu'en-tête personnalisé
#                         yield (b'--frame\r\n'
#                                b'Content-Type: image/jpeg\r\n' +
#                                f'X-bbox: {h}\r\n'.encode('utf-8') +
#                                f'X-dist_x: {val_send}\r\n'.encode('utf-8') +
#                                b'\r\n' +
#                                cropped + b'\r\n')
#                 else:
#                     print("I have nothing to send")     
#                 time.sleep(DELAY)
#         except GeneratorExit:
#             print("Client déconnecté.")
#         except Exception as e:
#             print(f"Erreur: {e}")
#         finally:
#             client_connected = False
#             camera.stop()
#             print("Arrêt de la caméra.")
#             ardu_talk(0, 0, CLIENT_NOT_CONNECTED_ID) # We tell the arduino that the client is disconnected
#             sys.exit()

#     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/fusion')
def fusion():
    def generate():
        global client_connected
        client_connected = True  # Le client est maintenant connecté
        try:
            while True:
                print("I'm getting a frame")
                
                try:
                    frame = camera.picam2.capture_array()
                except Exception as e:
                    print(f"Erreur lors de la capture d'une image : {e}")
                    continue
            
                # Détection des objets
                detections = camera.get_detections()
                labels = camera.get_labels()

                # Rassembler les données de détection
                detection_data = {"detections": []}
                list_persons = []

                for detection in detections:
                    if int(detection.category) > len(labels) - 1:
                        continue
                    
                    label = labels[int(detection.category)]
                    if label != "person":  # Filtre uniquement pour les personnes
                        continue
                    
                    x, y, w, h = detection.box
                    if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                        if y >= 0 and y + h <= frame.shape[0] and x >= 0 and x + w <= frame.shape[1]:
                            cropped_person = frame[y:y+h, x:x+w]
                            
                            # Encodage du recadré pour éviter la perte d'information
                            _, buffer = cv2.imencode('.jpg', cropped_person)
                            list_persons.append(buffer.tobytes())
                        else:
                            print(f"Erreur : Région de découpage invalide (x: {x}, y: {y}, w: {w}, h: {h}).")
                            continue
                    else:
                        print("Erreur : La frame est vide ou invalide.")
                        continue
                    
                    # Encodage du recadré pour éviter la perte d'information
                    
                    detection_data['detections'].append({
                        "label": label,
                        "bbox": [x, y, w, h]
                    })
            

                    for cropped in list_persons:
                        # Récupérer la valeur spécifique à envoyer
                        detections = detection_data.get("detections", [])
                        for detection in detections:
                            bbox = detection.get("bbox", [])
                            x, y, w, h = bbox

                            val_send = x+w/2 - center_x

                        # Ajouter la valeur en tant qu'en-tête personnalisé
                        print("je vais yield poto")
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n' +
                               f'X-bbox: {h}\r\n'.encode('utf-8') +
                               f'X-dist_x: {val_send}\r\n'.encode('utf-8') +
                               b'\r\n' +
                               cropped + b'\r\n')
                        print("J'ai fini de yield")
                        
                time.sleep(DELAY)
        except GeneratorExit:
            print("Client déconnecté.")
        except Exception as e:
            print(f"Erreur: {e}")
        finally:
            client_connected = False
            camera.stop()
            print("Arrêt de la caméra.")
            ardu_talk(0, 0, CLIENT_NOT_CONNECTED_ID) # We tell the arduino that the client is disconnected
            sys.exit()

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')




@app.route('/update_data', methods=['POST'])
def update_data():
    global client_connected, counter_lost
    print("I'm trying to read something")

    try:
        # Lire les données JSON envoyées
        data = request.json
        if 'x_distance' in data and 'height_box' in data and "tracking" in data:
            
            x_distance = float(data['x_distance'])
            height_box = int(data['height_box'])  
            tracking = int(data['tracking'])


            prof = calculate_distance(HUMAN_SIZE, abs(height_box))
            # print(f"Distance calculée : {prof:.2f} mètres")

            # angle = calculate_angle(x_distance, prof)
            mdist = CONVERSION_FACTOR * x_distance
            
            
            print("I'm trying to send to the arduino")
            
            
            
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
            
            track_send = smooth_tracking(tracking)
            ardu_talk(prof, mdist, track_send)
                
            print("I sent to the arduino")
            
            
        else:
            print("I read nothing bro") 

        return jsonify({"status": "success", "message": "Données reçues avec succès."}), 200
    
    except ValueError:
        # Gestion de l'erreur de conversion en entier
        return jsonify({"status": "error", "message": "'x_distance' doit être convertible en entier"}), 400
    
    except Exception as e:
        # Gestion d'autres erreurs
        return jsonify({"status": "error", "message": str(e)}), 500
    









if __name__ == "__main__":

    # threading.Thread(target=capture_frames_and_data, daemon=True).start()

    app.run(host='0.0.0.0', port=5000)
