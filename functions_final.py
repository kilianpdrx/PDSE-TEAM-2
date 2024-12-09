import cv2
import time
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

import torch


# Fonction pour obtenir les paramètres de la vidéo
def get_video_params(video_path):
    cap = cv2.VideoCapture(video_path)
    w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
    return cap, w, h, fps

# Fonction pour afficher et enregistrer les annotations
def annotate_frame(im0, annotations, annotator, center_x, center_y, w, h):
    for xyxy, mask, color, label, txt_color in annotations:
        mid_x = int((xyxy[0] + xyxy[2]) / 2)
        mid_y = int((xyxy[1] + xyxy[3]) / 2)
        
        
        cv2.circle(im0, (mid_x,mid_y), 5, (255, 0, 0), -1)  # Dessiner un cercle au centre de la boîte
        cv2.line(im0, (center_x, center_y), (mid_x,mid_y), (0, 0, 255), 2)  # Ligne de trajectoire (rouge)
        cv2.rectangle(im0, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
        annotator.seg_bbox(mask=mask, mask_color=color, label=label, txt_color=txt_color)
        

    # Lignes de trajectoire et de référence
    cv2.line(im0, (center_x, 0), (center_x, h), (0, 255, 0), 1)  # Ligne verticale (vert)
    cv2.line(im0, (0, center_y), (w, center_y), (0, 255, 0), 1)  # Ligne horizontale (vert)
    return im0


# Fonction pour afficher et calculer les FPS
def display_fps(im0, frame_processing_time, frame_jump):
    
    fps_display = 1 / frame_processing_time if frame_processing_time > 0 else -1
    fps_display = frame_jump * fps_display  # Mettre à jour le FPS en fonction du saut de frame
    cv2.putText(im0, f"FPS: {fps_display:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return im0


def calibrate(results, extractor, im0):
    """Calibrer en extrayant les caractéristiques d'une personne."""
    if results[0].boxes is None:
        return None, None

    boxes = results[0].boxes.xyxy.cpu().numpy()  # Coordonnées des boîtes
    if len(boxes) != 1:  # Assurez-vous qu'une seule personne est devant la caméra
        return None, None

    # Extraire les caractéristiques de l'image de la personne
    x1, y1, x2, y2 = map(int, boxes[0])
    cropped_person = im0[y1:y2, x1:x2]
    features = extractor(cropped_person)
    return features, cropped_person




def process_total(frame, results, extractor, target_features, annotator, center_x, threshold=0.8):
    """Comparer les caractéristiques des personnes détectées avec celles de la personne calibrée."""
    
    
    for result in results: 

        if result.boxes.id is None or result.masks is None:
            return []

        masks = result.masks.xy
        track_ids = result.boxes.id.int().cpu().tolist()
        classes = result.boxes.cls.int().cpu().tolist()
        confidences = result.boxes.conf.cpu().tolist()
        boxes = result.boxes.cpu().numpy()
        xyxys = boxes.xyxy
    

        annotations = []

        for mask, track_id, cls, conf, box_xyxy in zip(masks, track_ids, classes, confidences, xyxys):
            if cls == 0:
                x1, y1, x2, y2 = map(int, box_xyxy)
                cropped_person = frame[y1:y2, x1:x2]
                features = extractor(cropped_person)

                # Calcul de la similarité entre les caractéristiques extraites et celles de la cible
                similarity = torch.nn.functional.cosine_similarity(features, target_features).item()

                if similarity > threshold:  # Seuillage de la similarité
                    track_id = 1  # ID de la personne cible
                else:
                    track_id = -1  # Non ciblé

                # Annoter la frame
                mid_x = int((box_xyxy[0] + box_xyxy[2]) / 2)
                x_distance = mid_x - center_x
                color = colors(track_id, True)
                label = f"ID: {track_id}, Sim: {similarity:.2f}, x_dist: {x_distance:.2f}"
                txt_color = annotator.get_txt_color(color)
                
                annotations.append((box_xyxy, mask, color, label, txt_color))

    return annotations


def process_long_calib(frame, results, extractor, target_features, annotator, center_x, threshold=0.8):
    """Comparer les caractéristiques des personnes détectées avec celles de la personne calibrée."""
    
    
    for result in results: 

        if result.boxes.id is None or result.masks is None:
            return []

        masks = result.masks.xy
        track_ids = result.boxes.id.int().cpu().tolist()
        classes = result.boxes.cls.int().cpu().tolist()
        confidences = result.boxes.conf.cpu().tolist()
        boxes = result.boxes.cpu().numpy()
        xyxys = boxes.xyxy
    

        annotations = []

        for mask, track_id, cls, conf, box_xyxy in zip(masks, track_ids, classes, confidences, xyxys):
            if cls == 0:
                x1, y1, x2, y2 = map(int, box_xyxy)
                cropped_person = frame[y1:y2, x1:x2]
                features = extractor(cropped_person)


                for target_features in target_features:
                    if target_features is not None:
                        # Calcul de la similarité entre les caractéristiques extraites et celles de la cible
                        similarity = torch.nn.functional.cosine_similarity(features, target_features).item()

                        if similarity > threshold:  # Seuillage de la similarité
                            track_id = 1  # ID de la personne cible
                            break
                        else:
                            track_id = -1  # Non ciblé

                # Annoter la frame
                mid_x = int((box_xyxy[0] + box_xyxy[2]) / 2)
                x_distance = mid_x - center_x
                color = colors(track_id, True)
                label = f"ID: {track_id}, Sim: {similarity:.2f}, x_dist: {x_distance:.2f}"
                txt_color = annotator.get_txt_color(color)
                
                annotations.append((box_xyxy, mask, color, label, txt_color))

    return annotations
