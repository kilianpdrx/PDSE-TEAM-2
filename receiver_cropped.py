import cv2
import requests
import numpy as np


from FE_modif import FeatureExtractor2
from functions_final import *



def calibrate(extractor, cropped_person):
    # Détecter les caractéristiques de la personne
    target_features = extractor(cropped_person)


    return target_features


extractor = FeatureExtractor2(
    model_name='osnet_x0_25',
    model_path='osnet_x0_25_imagenet.pth',
    device='cpu'
)


target_features = None  # Caractéristiques de la personne calibrée
list_target_features = []  # Liste des caractéristiques de la personne calibrée
list_target_features2 = []  # Liste des caractéristiques de la personne calibrée

min_number_features = 30  # Nombre minimal de features pour la calibration
calibrated = False  # Statut de calibration
similarity_threshold = 0.75  # Seuil de similarité pour la comparaison de ReID

index_frame = 0 # on est obligés d'en mettre 2 parce qu'on saute des frames
frame_count = 0
total_processing_time = 0.0
start_time = time.time()
frame_jump = 1  # Ignorer certaines frames pour optimiser






# URL du flux vidéo, on peut mettre ça dans un navigateur pour voir le flux
url = {"cropped": "http://10.11.6.148:5000/cropped_feed",
       "full": "http://10.11.6.148:5000/full_feed"}

CHOIX_URL = "cropped" # or full


# Connexion au flux vidéo
stream = requests.get(url[CHOIX_URL], stream=True)
if stream.status_code != 200:
    print("Impossible de se connecter au flux vidéo.")
    exit()

# Variables pour extraire les frames
bytes_data = b""
frame_counter = 0


if CHOIX_URL == 1:
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk

        # Détecter la fin d'une image
        a = bytes_data.find(b'\xff\xd8')  # Début de l'image JPEG
        b = bytes_data.find(b'\xff\xd9')  # Fin de l'image JPEG


        if a != -1 and b != -1:
            # Extraire l'image JPEG
            index_frame += 1
            
            
            jpeg_data = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]

            # Convertir l'image en format numpy pour OpenCV
            nparr = np.frombuffer(jpeg_data, np.uint8)
            cropped_person = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            
            if not calibrated:
                # Étape de calibration
                target_features = calibrate(extractor, cropped_person)
                if target_features is not None:
                    print(f"Calibration : {index_frame/min_number_features * 100:.2f}%")
                    list_target_features.append(target_features)
                    
                else:
                    print("Veuillez vous placer devant la caméra pour la calibration.")
                        
                if len(list_target_features) >= min_number_features:
                    print("Calibration terminée. Personne cible enregistrée.")
                    calibrated = True
                    print("Durée de la calibration : ", time.time() - start_time)
                    # cv2.imshow("Personne calibrée", target_image)
                    # cv2.waitKey(0)
                    # cv2.destroyWindow("Personne calibrée")
            
            
            
            
            # if target_features is not None:
            #     print("Calibration réussie.")


            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow("Personne calibrée", cropped_person)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
else:
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk

        # Détecter la fin d'une image
        a = bytes_data.find(b'\xff\xd8')  # Début de l'image JPEG
        b = bytes_data.find(b'\xff\xd9')  # Fin de l'image JPEG


        if a != -1 and b != -1:
            # Extraire l'image JPEG
            index_frame += 1
            
            
            jpeg_data = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]

            # Convertir l'image en format numpy pour OpenCV
            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow("Full frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            

cv2.destroyAllWindows() 

