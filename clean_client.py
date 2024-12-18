
from client_utils import *

CHOIX_FLUX = [3] # 0: full, 1: cropped, 2: data 3: fusion
show_person = True
wait_for_input = True
IP_address = "128.179.209.34"
# IP_address = "172.20.10.6"
# http://128.179.209.34:5000/full_feed


client = Client(show_person, wait_for_input, IP_address)


if __name__ == "__main__":
    
    for t in CHOIX_FLUX:
        client.threads[t].start()

    # Thread principal pour afficher les flux et les données
    client.display_streams2()
