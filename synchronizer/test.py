from synchroniser import Subscriber
import time


subscriber= Subscriber()
#subscriber.set_max_messages(10)  # Receive 10 ticks

#for i in range(10):
subscriber.receive_messages()
    #time.sleep(5)
    #if subscriber.acknowledge_message():
      #  print("Acknowledged message")
    # if subscriber.message_count >= subscriber.max_messages:
    #     print("Reached message limit")
    #     break
    #time.sleep(0.1)  # Prevent busy wait

subscriber.close()  # Close the subscriber connection
print("Subscriber closed")

