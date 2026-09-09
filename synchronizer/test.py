from synchroniser import Subscriber
import time


subscriber= Subscriber()
#subscriber.set_max_messages(10)  # Receive 10 ticks

#while True:
subscriber.receive_messages()
if subscriber.message_count >= subscriber.max_messages:
    print("Reached message limit")
    #break
time.sleep(0.1)  # Prevent busy wait

