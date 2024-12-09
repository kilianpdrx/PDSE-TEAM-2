import cv2
import requests
import numpy as np


from FE_modif import FeatureExtractor2
from functions_final import *

extractor = FeatureExtractor2(
    model_name='osnet_x0_25',
    model_path='osnet_x0_25_imagenet.pth',
    device='cpu'
)
    
# URL du flux vidéo
url = "http://10.11.6.148:5000/video_feed"  # Remplacez par l'adresse de votre Raspberry Pi

# Connexion au flux vidéo
stream = requests.get(url, stream=True)
if stream.status_code != 200:
    print("Impossible de se connecter au flux vidéo.")
    exit()

# Variables pour extraire les frames
bytes_data = b""
frame_counter = 0

for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk

    # Détecter la fin d'une image
    a = bytes_data.find(b'\xff\xd8')  # Début de l'image JPEG
    b = bytes_data.find(b'\xff\xd9')  # Fin de l'image JPEG

    if a != -1 and b != -1:
        # Extraire l'image JPEG
        jpeg_data = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]

        # Convertir l'image en format numpy pour OpenCV
        nparr = np.frombuffer(jpeg_data, np.uint8)
        cropped_person = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        target_features, target_image = calibrate(extractor, cropped_person)

        # Afficher l'image dans une fenêtre distincte
        cv2.imshow(f"Cropped Person {frame_counter}", cropped_person)
        frame_counter += 1

        # Quittez avec la touche 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
