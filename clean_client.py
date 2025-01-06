
from client_utils import *

CHOIX_FLUX = [1] # 0: full, 1: fusion auto reconnect
show_person = True
wait_for_input = True
IP_address = "128.179.209.34"

# http://128.179.209.34:5000/full_feed


client = Client(show_person, wait_for_input, IP_address)


if __name__ == "__main__":
    
    # to start the threads
    for t in CHOIX_FLUX:
        client.threads[t].start()

    # main function
    client.display_streams()
