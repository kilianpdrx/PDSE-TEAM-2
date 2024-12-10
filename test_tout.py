import cv2
import requests
import numpy as np
import threading
import queue
import time
import json


from FE_modif import FeatureExtractor2
from functions_final import *


center_x = 320
center_y = 240
sim_threshold = 0.7
IDED_PERSON = 1
BAD_PERSON = -1



def calibrate(extractor, cropped_person):
    # Détecter les caractéristiques de la personne
    target_features = extractor(cropped_person)
    return target_features


def compare(frame, extractor, target_features):
    """Comparer les caractéristiques des personnes détectées avec celles de la personne calibrée."""
    
    features = extractor(frame)
    mean = 0
    x_distance = 0
    
    # si la moyenne des similarités est supérieure au seuil, on considère que la personne est ciblée
    for target_features in target_features:
        if target_features is not None:
            # Calcul de la similarité entre les caractéristiques extraites et celles de la cible
            similarity = torch.nn.functional.cosine_similarity(features, target_features).item()
            mean += similarity

    if mean/len(target_features) > sim_threshold:
        track_id = IDED_PERSON
        x_distance = mid_x - center_x # la faut récupérer mid_x avec le flux data
    else:
        track_id = BAD_PERSON
        x_distance = BAD_PERSON

    return track_id, x_distance


extractor = FeatureExtractor2(
    model_name='osnet_x0_25',
    model_path='osnet_x0_25_imagenet.pth',
    device='cpu'
)



# URL des flux
url = {
    "cropped": "http://10.11.6.148:5000/cropped_feed",
    "full": "http://10.11.6.148:5000/full_feed",
    "data": "http://10.11.6.148:5000/data"
}

# Queues pour synchronisation des données
full_frame_queue = queue.Queue(maxsize=10)
cropped_frame_queue = queue.Queue(maxsize=10)
data_queue = queue.Queue(maxsize=10)


CHOIX_FLUX = [1,2] # 0: full, 1: cropped, 2: data
DO_CALIBRATION = True
target_features = None  # Caractéristiques de la personne calibrée
list_target_features = []  # Liste des caractéristiques de la personne calibrée
min_number_features = 30  # Nombre minimal de features pour la calibration
calibrated = False  # Statut de calibration


# Fonctions pour chaque flux
def fetch_full_feed():
    """Thread pour le flux vidéo complet."""
    stream = requests.get(url["full"], stream=True)
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
    stream = requests.get(url["cropped"], stream=True)
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
    stream = requests.get(url["data"], stream=True)
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

# Thread principal pour afficher les images et les données
def display_streams():
    global calibrated
    """Affiche les deux flux vidéo et les données JSON dans le terminal."""
    while True:
        # Afficher les frames complètes dans une fenêtre OpenCV
        if not full_frame_queue.empty():
            full_frame = full_frame_queue.get()
            cv2.imshow("Full Feed", full_frame)

        # Afficher les images recadrées dans une autre fenêtre OpenCV
        if not cropped_frame_queue.empty():
            cropped_frame = cropped_frame_queue.get()
            # cv2.imshow("Cropped Feed", cropped_frame)
            
            if DO_CALIBRATION:
                if not calibrated:
                    target_features = calibrate(extractor, cropped_frame)
                    if target_features is not None:
                        list_target_features.append(target_features)
                        print(f"Calibration : {len(list_target_features) / min_number_features * 100:.2f}%")

                    if len(list_target_features) >= min_number_features:
                        print("Calibration terminée.")
                        calibrated = True
                
                # else:
                #     # Comparer les caractéristiques de la personne calibrée avec celles des personnes détectées

                
                cv2.imshow("Cropped Person", cropped_frame)
                





        # Afficher les données JSON dans le terminal
        if not data_queue.empty():
            data = data_queue.get()
            print("Données reçues :", json.dumps(data, indent=4))

        # Fermer les fenêtres si 'q' est pressé
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        
        
        
    cv2.destroyAllWindows()

# Démarrer les threads
threads = [
    threading.Thread(target=fetch_full_feed, daemon=True),
    threading.Thread(target=fetch_cropped_feed, daemon=True),
    threading.Thread(target=fetch_data_feed, daemon=True)
]

for t in CHOIX_FLUX:
    threads[t].start()

# Thread principal pour afficher les flux et les données
display_streams()
