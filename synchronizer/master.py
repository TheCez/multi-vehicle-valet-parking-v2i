import json
import zmq
import time
import uuid



class Master:


    no_of_subscribers = 0 # Static variable to keep track of subscribers
    subscribers = []  # List to keep track of subscriber instances
    

    def __init__(self, pub_port=5555, sync_port=5556):
        self.context = zmq.Context()
        
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        
        # Sync socket for acknowledgments
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        
        # # Wait for subscriber to connect
        # print("Waiting for subscriber...")
        # self.sync_socket.recv_string()
        # self.sync_socket.send_string("Ready")
        # print("Subscriber connected")

    def broadcast_tick(self):
        try:
            # Send a tick message to all subscribers
            tick_message = json.dumps({"command": "TICK"})
            self.pub_socket.send_string(tick_message)
            print("Tick broadcasted")
            
            # Wait for acknowledgment
            self.verify_acknowledgment()
            #ack = self.sync_socket.recv_string()
            #self.sync_socket.send_string("Tick acknowledged")

            return True
            
        except Exception as e:
            print(f"Error broadcasting tick: {e}")
            return False
    
    def verify_acknowledgment(self):
        self.waiting_for_ack = self.no_of_subscribers
        # Wait for all subscribers to acknowledge the tick
        while self.waiting_for_ack > 0:
            try:
                # Wait for acknowledgment from a subscriber
                ack = self.sync_socket.recv_string()
                print(f"Acknowledgment received: {ack}")
                
                # Send acknowledgment back to the subscriber
                self.sync_socket.send_string("ACK")
                
                # Decrease the count of waiting acknowledgments
                self.waiting_for_ack -= 1
                
            except zmq.Again:
                # No message received, continue waiting
                time.sleep(0.1)


    # def send_data(self, json_data):
    #     try:
    #         # Convert the geometry data to JSON string
    #         message = json_data
    #         self.pub_socket.send_string(json.dumps(message))
    #         # print(message)

    #         print(f"Sent {message.get('command')}")
            
    #         # Wait for acknowledgment
    #         ack = self.sync_socket.recv_string()
    #         self.sync_socket.send_string("Next")
            
    #         return True
    #     except Exception as e:
    #         print(f"Error sending {message.get('command')}: {e}")
    #         return False

    def pass_baton(self):
        try:
            # Send a baton message to all subscribers
            baton_message = json.dumps({"command": "BATON"})
            self.pub_socket.send_string(baton_message)
            print("Baton passed")
            
            # Wait for acknowledgment
            ack = self.sync_socket.recv_string()
            self.sync_socket.send_string("Baton acknowledged")
            
            return True
            
        except Exception as e:
            print(f"Error passing baton: {e}")
            return False
        
    def send_termination_signal(self):
        try:
            # Send special termination message
            termination_message = json.dumps({"command": "TERMINATE"})
            self.pub_socket.send_string(termination_message)
            
            # Wait for final acknowledgment
            ack = self.sync_socket.recv_string()
            self.sync_socket.send_string("Terminate")
            print("Termination signal sent and acknowledged")
            
        except Exception as e:
            print(f"Error sending termination signal: {e}")
            
    def close(self):
        try:
            self.send_termination_signal()
            time.sleep(1)  # Give time for the signal to be processed
            self.pub_socket.close()
            self.sync_socket.close()
            self.context.term()
            
        except Exception as e:
            print(f"Error during close: {e}")


class Subscriber:
    #subscriber_id =  None # Unique ID for each subscriber instance

    def __init__(self, sub_port=5555, sync_port=5556):
        self.context = zmq.Context()
        
        # Subscriber socket
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        # Sync socket for acknowledgments
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        self.running = True
        self.termination_event = zmq.Event()
        self.receive_queue = zmq.Queue()
        self.no_of_subscribers = 0
        self.no_of_subscribers += 1
        Master.no_of_subscribers += 1
        self.subscriber_id = str(uuid.uuid4())  # Generate a unique ID for this subscriber
        Master.subscribers.append(self.subscriber_id)  # Add this subscriber to the master list
        print(f"Subscriber connected. Total subscribers: {Master.no_of_subscribers}")
        # Send ready signal to master
        print("Sending ready signal to master...")
        self.sync_socket.send_string("Ready")
        response = self.sync_socket.recv_string()
        print(f"Received response from master: {response}")

    def receive_messages(self):
        print("Starting to receive messages...")
        while self.running and not self.termination_event.is_set():
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                if message == "TICK":
                    #self.acknowledge_message()
                    print(f"Received message: {message}")
                    return message
                #self.receive_queue.put(message)
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as e:
                print(f"Error in receive_messages: {e}")
                self.running = False
                break

    def acknowledge_message(self):
        self.sync_socket.send_string("ACK")  

    def send_termination_signal(self):
        try:
            # Send termination signal to master
            self.sync_socket.send_string("TERMINATE")
            response = self.sync_socket.recv_string()
            print(f"Received response from master: {response}")
            self.termination_event.set()  # Signal to stop receiving messages
            self.running = False
        except Exception as e:
            print(f"Error sending termination signal: {e}")
    def close(self):
        try:
            self.send_termination_signal()
            time.sleep(1)  # Give time for the signal to be processed
            self.sub_socket.close()
            self.sync_socket.close()
            self.context.term()
            Master.no_of_subscribers -= 1
            print(f"Subscriber disconnected. Remaining subscribers: {Master.no_of_subscribers}")
        except Exception as e:
            print(f"Error during close: {e}")
# Example usage:
if __name__ == "__main__":
    master = Master()
    subscriber = Subscriber()

    # Simulate sending data
    for i in range(5):
        master.broadcast_tick()
    # Simulate receiving messages
        subscriber.receive_messages()
        time.sleep(1)

    # Broadcast tick
    master.broadcast_tick()

    # Close connections
    subscriber.close()
    master.close()


