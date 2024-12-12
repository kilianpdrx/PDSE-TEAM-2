import cv2
import requests
import numpy as np
import threading
import queue
import time
import json


from FE_modif import FeatureExtractor2
from client_utils import *



extractor = FeatureExtractor2(
    model_name='osnet_x0_25',
    model_path='osnet_x0_25_imagenet.pth',
    device='cpu'
)



# URL des flux
urls = {
    "cropped": "http://10.11.6.148:5000/cropped_feed",
    "full": "http://10.11.6.148:5000/full_feed",
    "data": "http://10.11.6.148:5000/data",
    "fusion": "http://10.11.6.148:5000/fusion"
}

# Queues pour synchronisation des données
full_frame_queue = queue.Queue(maxsize=1)
cropped_frame_queue = queue.Queue(maxsize=1)
data_queue = queue.Queue(maxsize=1)
final_queue = queue.Queue(maxsize=1)




center_x = 320
center_y = 240
sim_threshold = 0.7
IDED_PERSON = 1
BAD_PERSON = -1
DELAY = 0.01


DO_CALIBRATION = True
target_features = None  # Caractéristiques de la personne calibrée
list_target_features = []  # Liste des caractéristiques de la personne calibrée
min_number_features = 30  # Nombre minimal de features pour la calibration
calibrated = False  # Statut de calibration

CHOIX_FLUX = [3] # 0: full, 1: cropped, 2: data 3: fusion


POST_URL = "http://10.11.6.148:5000/update_data"

def send_data_to_server(data):
    try:
        response = requests.post(POST_URL, json=data)
        if response.status_code == 200:
            # print("Données envoyées avec succès.")
            pass
        else:
            print(f"Erreur lors de l'envoi : {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Erreur : {e}")



# Fonctions pour chaque flux
def fetch_full_feed():
    """Thread pour le flux vidéo complet."""
    stream = requests.get(urls["full"], stream=True)
    if stream.status_code != 200:
        print("Impossible de se connecter au flux full_feed.")
        return

    bytes_data = b""
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')

        if a != -1 and b != -1:
            jpeg_data = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]

            # Convertir en numpy array pour OpenCV
            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            full_frame_queue.put(frame)  # Ajouter à la queue

def fetch_cropped_feed():
    """Thread pour le flux des images recadrées."""
    stream = requests.get(urls["cropped"], stream=True)
    if stream.status_code != 200:
        print("Impossible de se connecter au flux cropped_feed.")
        return

    bytes_data = b""
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')

        if a != -1 and b != -1:
            jpeg_data = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]

            # Convertir en numpy array pour OpenCV
            nparr = np.frombuffer(jpeg_data, np.uint8)
            cropped_person = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cropped_frame_queue.put(cropped_person)  # Ajouter à la queue

def fetch_data_feed():
    """Thread pour le flux JSON."""
    stream = requests.get(urls["data"], stream=True)
    if stream.status_code != 200:
        print("Impossible de se connecter au flux data.")
        return

    for line in stream.iter_lines():
        if line:
            try:
                data = json.loads(line.decode('utf-8').replace("data: ", ""))
                data_queue.put(data)  # Ajouter à la queue
            except json.JSONDecodeError:
                print("Erreur lors de la lecture du flux JSON.")




def fetch_final_flux():
    stream = requests.get(urls["fusion"], stream=True)
    if stream.status_code == 200:
        print("Connexion établie avec le serveur.")

        byte_buffer = b''
        boundary = b'--frame'
    
        try:
            for chunk in stream.iter_content(chunk_size=1024):
                byte_buffer += chunk

                while boundary in byte_buffer:
                    # Trouver les délimitations des frames
                    frame_start = byte_buffer.find(boundary)
                    frame_end = byte_buffer.find(boundary, frame_start + len(boundary))

                    if frame_end == -1:
                        break

                    # Extraire une seule frame avec ses en-têtes
                    frame = byte_buffer[frame_start:frame_end]
                    byte_buffer = byte_buffer[frame_end:]

                    # Séparer les en-têtes du contenu de l'image
                    header_end = frame.find(b'\r\n\r\n')
                    headers = frame[:header_end].decode('utf-8')
                    image_data = frame[header_end + 4:]

                    # Lire les en-têtes personnalisés
                    height_box = None
                    dist_x = None
                    for line in headers.split('\r\n'):
                        if line.startswith('X-bbox:'):
                            height_box = line.split(':', 1)[1].strip()
                        elif line.startswith('X-dist_x:'):
                            dist_x = line.split(':', 1)[1].strip()

                    # Décoder l'image
                    image_array = np.frombuffer(image_data, dtype=np.uint8)
                    frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                    final_queue.put((frame, dist_x, height_box))
                    
                    # if frame is not None:
                    #     # Afficher l'image et les métadonnées
                    #     print(f"Image ID: {image_id}, Extra Value: {extra_value}")
                    #     cv2.imshow('Frame', frame)

                    #     # Arrêter la boucle si 'q' est pressé
                    #     if cv2.waitKey(1) & 0xFF == ord('q'):
                    #         break
        except Exception as e:
            print(f"Erreur lors du traitement : {e}")
        finally:
            cv2.destroyAllWindows()
    
    else:
        print(f"Erreur : impossible de se connecter au serveur. Code {stream.status_code}")



def display_streams2():
    global calibrated
    """Affiche les deux flux vidéo et les données JSON dans le terminal."""
    while True:
        cropped_frame = None
        data = None
        
        if not final_queue.empty():
            cropped_frame, x_dist, height_box = final_queue.get()

        
        # Calibration
        if DO_CALIBRATION:
            if cropped_frame is not None:
                if not calibrated:
                    target_features = calibrate(extractor, cropped_frame)
                    if target_features is not None:
                        list_target_features.append(target_features)
                        print(f"Calibration : {len(list_target_features) / min_number_features * 100:.2f}%")

                    if len(list_target_features) >= min_number_features:
                        print("Calibration terminée.")
                        calibrated = True
                else:
                    tracking = compare2(cropped_frame, extractor, list_target_features, sim_threshold)
                    data_to_send = {
                        "x_distance": x_dist,
                        "height_box": height_box, 
                        "tracking": tracking
                    }
                    send_data_to_server(data_to_send)
                    # print(f"Distance en x : {x_dist}")
            
                cv2.imshow("Cropped Person", cropped_frame)
            # else:
            #     print("Image recadrée non disponible.")    

        time.sleep(DELAY)
        # Fermer les fenêtres si 'q' est pressé
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()



# Démarrer les threads
threads = [
    threading.Thread(target=fetch_full_feed, daemon=True),
    threading.Thread(target=fetch_cropped_feed, daemon=True),
    threading.Thread(target=fetch_data_feed, daemon=True),
    threading.Thread(target=fetch_final_flux, daemon=True)
]



if __name__ == "__main__":
    
    for t in CHOIX_FLUX:
        threads[t].start()

    # Thread principal pour afficher les flux et les données
    display_streams2()
