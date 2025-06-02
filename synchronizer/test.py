from synchroniser import Subscriber
import time

# Create a Subscriber instance
subscriber = Subscriber()

# Set the number of messages to process
no_of_messages = 10
messages_processed = 0

# Loop to control the number of messages
while messages_processed < no_of_messages and subscriber.running:
    if subscriber.receive_messages():
        # Simulate a random task with a 5-second delay
        #time.sleep(5)
        # Acknowledge the message
        if subscriber.acknowledge_message():
            messages_processed += 1
            print(f"Processed message {messages_processed}/{no_of_messages}")
    # Optional: Add a small delay to prevent tight CPU loop if no message is received
    else:
        time.sleep(0.1)

# Close the subscriber connection
subscriber.close()
print("Subscriber closed")