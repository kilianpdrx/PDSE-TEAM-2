from ai_camera import IMX500Detector
from serveur_utils import *


from ai_camera import IMX500Detector


model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)
camera.start(show_preview=False)



if __name__ == "__main__":
    serveur = Serveur(camera)
    threading.Thread(target=serveur.capture_frames_and_data, daemon=True).start()

    serveur.run()