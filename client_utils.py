import cv2
import time
import requests
import numpy as np
import threading
import queue
import torch
from screeninfo import get_monitors


from FE_modif import FeatureExtractor2


monitor = get_monitors()[0]
screen_height = monitor.height
half_screen_height = int(screen_height // 1.5)


IDED_PERSON = 1
BAD_PERSON = -1

target_features = None  # Characteristics of the calibrated person
list_target_features = []  # List of calibrated person characteristics
min_number_features = 30  # Minimum number of features for calibration
calibrated = False  # Calibration status
DELAY = 0.05    
SIM_THRESHOLD = 0.7


index_frame = 0


def calibrate(extractor, cropped_person):
    """ This function is used to calibrate the system by detecting the characteristics of the person to be tracked.

    Args:
        extractor : the model used to extract the characteristics of the person
        cropped_person (np.ndarray) : the image of the person to be tracked

    Returns:
        (list): the list of characteristics of the person to be tracked for this frame
    """
    global index_frame
    # we skip some frames to let the person turn on himself
    index_frame += 1
    
    if index_frame % 50 == 0:
        return None
    
    # Detecting the person's characteristics
    target_features = extractor(cropped_person)
    return target_features


def compare2(frame, extractor, list_target_features):
    """ This function is used to compare the characteristics of the detected persons with those of the calibrated person.

    Args:
        frame (np.ndarray) : the image of the person to be compared
        extractor : the model used to extract the characteristics of the person
        list_target_features (list) : the list of characteristics of the calibrated person

    Returns:
        int : +1 if the person is identified, -1 otherwise
    """
    # Compare the characteristics of the detected persons with those of the calibrated person.
    
    features = extractor(frame)
    mean = 0

    
    # if the average similarity is greater than the threshold, the person is considered to be targeted
    for target_features in list_target_features:
        if target_features is not None:
            # Calculation of similarity between extracted and target features
            similarity = torch.nn.functional.cosine_similarity(features, target_features).item()
            mean += similarity
    
    total_val = mean/len(list_target_features)

    if total_val > SIM_THRESHOLD:
        print(f"Person identified {total_val:.2f}")
        track_id = IDED_PERSON
    else:
        print(f"Person non identified {total_val:.2f}")
        track_id = BAD_PERSON

    return track_id





class Client:
    def __init__(self, show_person=False, wait_for_input=True, IP_address=""):
        
        self.list_target_features = [] # List of calibrated person characteristics
        self.IP = IP_address # IP address of the server
        
        self.urls = {
            "full": f"http://{self.IP}:5000/full_feed",
            "fusion": f"http://{self.IP}:5000/fusion"
        }
        self.POST_URL = f"http://{self.IP}:5000/update_data"
        
        self.full_frame_queue = queue.Queue(maxsize=1) # Queue for the full feed
        self.show_cropped = show_person # Whether to display the cropped person
        self.wait_for_input = wait_for_input # Whether to wait for input after the calibration
        
        # Load the feature extraction model
        self.extractor = FeatureExtractor2(
            model_name='osnet_x0_25',
            model_path='osnet_x0_25_imagenet.pth',
            device='cpu'
        )
    
        # Threads for the video streams
        self.threads = [
            threading.Thread(target=self.fetch_full_feed, daemon=True),
            threading.Thread(target=self.fetch_final_flux_auto, daemon=True)
        ]
        
        # the last frame received
        self.latest_frame = None
        self.lock = threading.Lock()
    
    
    
    
    def send_data_to_server(self,data):
        """ This function is used to send data to the server.

        Args:
            data (dict): the data to be sent
        """
        
        try:
            response = requests.post(self.POST_URL, json=data)
            if response.status_code == 200:
                pass
            else:
                print(f"Erreur lors de l'envoi : {response.status_code}, {response.text}")
        except Exception as e:
            print(f"Erreur : {e}")
    
    def fetch_full_feed(self):
        """ Thread for the complete video stream. """
        
        stream = requests.get(self.urls["full"], stream=True)
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

                # Convert to numpy array for OpenCV
                nparr = np.frombuffer(jpeg_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                self.full_frame_queue.put(frame)

    def fetch_final_flux_auto(self):
        """ Thread for fusion flow with automatic reconnection. """
        while True:
            try:
                stream = requests.get(self.urls["fusion"], stream=True, timeout=20)
                if stream.status_code != 200:
                    print("Impossible de se connecter au flux fusion.")
                    time.sleep(2)  # wait before retrying
                    continue

                byte_buffer = b''
                boundary = b'--frame'
                print("Début de la lecture du flux fusion.")
                for chunk in stream.iter_content(chunk_size=1024):
                    byte_buffer += chunk

                    while boundary in byte_buffer:
                        # find the beginning and end of the frame
                        frame_start = byte_buffer.find(boundary)
                        frame_end = byte_buffer.find(boundary, frame_start + len(boundary))

                        if frame_end == -1:
                            break

                        # Extract a single frame with its headers
                        frame = byte_buffer[frame_start:frame_end]
                        byte_buffer = byte_buffer[frame_end:]

                        # Separate headers from image content
                        header_end = frame.find(b'\r\n\r\n')
                        headers = frame[:header_end].decode('utf-8')
                        image_data = frame[header_end + 4:]

                        # Read custom headers
                        height_box = None
                        dist_x = None
                        for line in headers.split('\r\n'):
                            if line.startswith('X-bbox:'):
                                height_box = line.split(':', 1)[1].strip()
                            elif line.startswith('X-dist_x:'):
                                dist_x = line.split(':', 1)[1].strip()

                        # Decode the image
                        image_array = np.frombuffer(image_data, dtype=np.uint8)
                        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        with self.lock:
                            self.latest_frame = (frame, dist_x, height_box)
                        
            except requests.RequestException as e:
                print(f"Erreur de connexion au flux fusion : {e}")
                time.sleep(2)  # wait before retrying
            except Exception as e:
                print(f"Erreur inattendue : {e}")
                time.sleep(2)  # wait before retrying

    def display_streams(self):
        """ This function is used to calibrate the system, compare the persons and display the video streams. """
        
        global calibrated
        while True:
            cropped_frame = None
            
            with self.lock:
                latest_frame = self.latest_frame
            
            if latest_frame is not None:
                # Decompose the tuple if it is not None
                cropped_frame, x_dist, height_box = latest_frame

                # Calibration
                if cropped_frame is not None:
                    
                    # no one is on the frame but we still display it
                    if x_dist == -1 or height_box == -1:
                        continue
                    else:
                        if not calibrated:
                            target_features = calibrate(self.extractor, cropped_frame)
                            if target_features is not None:
                                self.list_target_features.append(target_features)
                                print(f"Calibration : {len(self.list_target_features) / min_number_features * 100:.2f}%")
                            else: continue # skip the rest of the loop
                            
                            if len(self.list_target_features) >= min_number_features:
                                print("Calibration terminée.")
                                calibrated = True
                                
                                if self.wait_for_input:
                                    input("Press Enter to continue...")
                            
                        else:
                            tracking = compare2(cropped_frame, self.extractor, self.list_target_features)
                            data_to_send = {
                                "x_distance": x_dist,
                                "height_box": height_box, 
                                "tracking": tracking
                            }
                            self.send_data_to_server(data_to_send)
                
                if self.show_cropped:
                    # to display the cropped person with a good aspect ratio
                    cropped_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2RGB)
                    height, width, _ = cropped_frame.shape
                    aspect_ratio = width / height
                    new_height = half_screen_height
                    new_width = int(new_height * aspect_ratio)
                    resized_frame = cv2.resize(cropped_frame, (new_width, new_height))
                    
                    cv2.imshow("Cropped Person", resized_frame)

            time.sleep(DELAY)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()








