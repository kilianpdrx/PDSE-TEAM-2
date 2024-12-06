import cv2
import time
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from torchreid.utils import FeatureExtractor
import torch
from functions_final import *


def main():
    
    video_path = 0  # 0 pour la webcam
    cap, w, h, fps = get_video_params(video_path)
    out = cv2.VideoWriter("lessgo.mp4", cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    # Initialisation des modèles YOLO et ReID
    yolo_seg = YOLO("yolo11n-seg.pt")
    extractor = FeatureExtractor(
        model_name='osnet_x1_0',
        model_path='osnet_x1_0_imagenet.pth',
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


    # Boucle de traitement de chaque frame de la vidéo
    while True:
        ret, im0 = cap.read()
        if not ret:
            print("La vidéo est terminée ou vide.")
            break
        
        # Ignorer certaines frames pour la performance
        if frame_count % frame_jump != 0:
            frame_count += 1
            continue

        frame_start_time = time.time() 
        annotator = Annotator(im0, line_width=2)
        
        
        results = yolo_seg.track(im0, classes=[0])  # Détecter uniquement les personnes

        if not calibrated:
            # Étape de calibration
            target_features, target_image = calibrate(results, extractor, im0)
            if target_features is not None:
                print(f"Calibration : {index_frame/min_number_features * 100:.2f}%")
                list_target_features.append(target_features)
                
            else:
                print("Veuillez vous placer devant la caméra pour la calibration.")
                    
            if len(list_target_features) >= min_number_features:
                print("Calibration terminée. Personne cible enregistrée.")
                calibrated = True
                # cv2.imshow("Personne calibrée", target_image)
                # cv2.waitKey(0)
                # cv2.destroyWindow("Personne calibrée")
                
        else: # une fois que la calibration est terminée
            annotations = process_long_calib(im0, results, extractor, list_target_features, annotator, w // 2, similarity_threshold)
            im0 = annotate_frame(im0, annotations, annotator, w // 2, h // 2, w, h)

        
        
        # Calcul du temps de traitement et des FPS pour la frame actuelle 
        frame_processing_time = time.time() - frame_start_time
        total_processing_time += frame_processing_time  
        im0 = display_fps(im0, frame_processing_time, frame_jump)
        
        out.write(im0)
        cv2.imshow("Person ReID Tracking", im0)
        frame_count += 1
        index_frame += 1

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Libération des ressources
    out.release()
    cap.release()
    cv2.destroyAllWindows()
    
    # Temps total de traitement de la vidéo
    end_time = time.time()
    total_time = end_time - start_time  
    average_processing_time = total_processing_time / frame_count if frame_count > 0 else -1 

    print(" ")
    print(f"Temps total de traitement de la vidéo : {total_time:.2f} secondes")
    print(f"Temps moyen de traitement par image : {average_processing_time:.4f} secondes")
    print(f"FPS moyen : {1 / average_processing_time:.2f} FPS")




if __name__ == "__main__":
    main()