import json
import zmq
import time
import uuid
import carla
import threading



class Master:


    no_of_subscribers = 0 # Static variable to keep track of subscribers
    #subscribers = []  # List to keep track of subscriber instances
    

    def __init__(self, pub_port=5555, sync_port=5556):
        self.context = zmq.Context()
        
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)  # Enable verbose mode to get subscription/unsubscription messages
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        
        # Sync socket for acknowledgments
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")

        # Use a poller to handle incoming subscription messages without blocking
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)
        
        # # Wait for subscriber to connect
        # print("Waiting for subscriber...")
        # self.sync_socket.recv_string()
        # self.sync_socket.send_string("Ready")
        # print("Subscriber connected")

        # Start background thread for polling subscription events
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()
        print("Background polling thread started for subscriber tracking.")


    def poll_subscriptions(self):
        """Background method to poll for subscription and unsubscription events."""
        while True:
            events = dict(self.poller.poll(1000))  # 1000ms timeout to prevent tight CPU loop
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                message = self.pub_socket.recv_multipart()
                if message:
                    # Check if it's a subscription (b'\x01') or unsubscription (b'\x00') message
                    if message[0][0:1] == b'\x01':
                        Master.no_of_subscribers += 1
                        print(f"New subscriber connected. Total subscribers: {Master.no_of_subscribers}")
                    elif message[0][0:1] == b'\x00':
                        Master.no_of_subscribers -= 1
                        print(f"Subscriber disconnected. Total subscribers: {Master.no_of_subscribers}")
            time.sleep(0.1)  # Small sleep to reduce CPU usage

    def get_subscriber_count(self):
        """Helper method to retrieve the current number of subscribers."""
        return Master.no_of_subscribers

    def broadcast_tick(self):
        # try:
            # Send a tick message to all subscribers
            #tick_message = json.dumps({"command": "TICK"})
            self.pub_socket.send_string("TICK")
            print("Tick broadcasted")
            
            # Wait for acknowledgment
            if self.no_of_subscribers > 0:
                self.verify_acknowledgment()
            #ack = self.sync_socket.recv_string()
            #self.sync_socket.send_string("Tick acknowledged")

            return True
            
        # except Exception as e:
        #     print(f"Error broadcasting tick: {e}")
        #     return False
    
    def verify_acknowledgment(self):
        self.waiting_for_ack = self.no_of_subscribers
        timeout = 2.0  # Timeout in seconds for all acknowledgments
        start_time = time.time()
        
        print(f"Waiting for {self.waiting_for_ack} acknowledgments...")
        # Wait for all subscribers to acknowledge the tick
        while self.waiting_for_ack > 0 and (time.time() - start_time) < timeout:
            try:
                # Attempt to receive acknowledgment from a subscriber without blocking
                ack = self.sync_socket.recv_string(flags=zmq.NOBLOCK)
                if ack == "ACK":
                    print(f"Acknowledgment received from subscriber. Remaining: {self.waiting_for_ack - 1}")
                # Send a reply to complete the REP socket cycle for this subscriber
                self.sync_socket.send_string("ACK_RECEIVED")
                # Decrease the count of waiting acknowledgments
                self.waiting_for_ack -= 1
            except zmq.Again:
                # No message received, wait briefly before retrying
                time.sleep(0.01)
            except zmq.ZMQError as e:
                print(f"ZMQ error during acknowledgment: {e}")
                break
        
        if self.waiting_for_ack > 0:
            print(f"Timeout reached. Still waiting for {self.waiting_for_ack} acknowledgments.")
        else:
            print("All acknowledgments received.")


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

    # def pass_baton(self):
    #     try:
    #         # Send a baton message to all subscribers
    #         baton_message = json.dumps({"command": "BATON"})
    #         self.pub_socket.send_string(baton_message)
    #         print("Baton passed")
            
    #         # Wait for acknowledgment
    #         ack = self.sync_socket.recv_string()
    #         self.sync_socket.send_string("Baton acknowledged")
            
    #         return True
            
    #     except Exception as e:
    #         print(f"Error passing baton: {e}")
    #         return False
        
    # def send_termination_signal(self):
    #     try:
    #         # Send special termination message
    #         termination_message = json.dumps({"command": "TERMINATE"})
    #         self.pub_socket.send_string(termination_message)
            
    #         # # Wait for final acknowledgment
    #         # ack = self.sync_socket.recv_string()
    #         # self.sync_socket.send_string("Terminate")
    #         # print("Termination signal sent and acknowledged")
            
    #     except Exception as e:
    #         print(f"Error sending termination signal: {e}")
            
    # def close(self):
    #     try:
    #         self.send_termination_signal()
    #         time.sleep(1)  # Give time for the signal to be processed
    #         self.pub_socket.close()
    #         self.sync_socket.close()
    #         self.context.term()
            
    #     except Exception as e:
    #         print(f"Error during close: {e}")


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
        #self.termination_event = zmq.Event()
        #self.receive_queue = zmq.QUEUE()
        #self.no_of_subscribers = 0
        #self.no_of_subscribers += 1
        #Master.no_of_subscribers += 1
        #self.subscriber_id = str(uuid.uuid4())  # Generate a unique ID for this subscriber
        #Master.subscribers.append(self.subscriber_id)  # Add this subscriber to the master list
        #print(f"Subscriber connected. Total subscribers: {Master.no_of_subscribers}")
        # Send ready signal to master
        # print("Sending ready signal to master...")
        # self.sync_socket.send_string("Ready")
        # response = self.sync_socket.recv_string()
        # print(f"Received response from master: {response}")

    def receive_messages(self):
        print("Starting to receive messages...")
        while self.running:
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                if message == "TICK":
                    print(f"Received message: {message}")
                    # Acknowledge the tick message
                    self.acknowledge_message()
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as e:
                print(f"Error in receive_messages: {e}")
                self.running = False
                break

    def acknowledge_message(self):
        print("Acknowledging message...")
        try:
            # Send acknowledgment to the server
            self.sync_socket.send_string("ACK")
            # Receive the server's reply to complete the REQ-REP cycle
            reply = self.sync_socket.recv_string(flags=zmq.NOBLOCK)
            print(f"Received reply from server: {reply}")
        except zmq.Again:
            print("Timeout or no reply from server during acknowledgment.")
            # Optionally retry or handle failure
        except zmq.ZMQError as e:
            print(f"Error during acknowledgment: {e}")

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

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # Optional: set fixed time step
    world.apply_settings(settings)
    client.set_timeout(10.0)
    world = client.get_world()

    #master = Master()

    for _ in range(100):
        world.tick()
        print("CARLA world ticked")
        print(master.no_of_subscribers)
        master.broadcast_tick()
        time.sleep(1)
    #subscriber = Subscriber()

    # # Simulate sending data
    # for i in range(5):
    #     master.broadcast_tick()
    # # Simulate receiving messages
    #     #subscriber.receive_messages()
    #     time.sleep(1)

    # # Broadcast tick
    # master.broadcast_tick()

    # Close connections
    #subscriber.close()
    master.close()


