import json
import zmq
import time
import uuid
import carla
import threading





class Master:
    no_of_subscribers = 0  # Static variable to keep track of subscribers
    pending_disconnects = 0  # Track pending disconnects to adjust after acknowledgment

    def __init__(self, pub_port=5555, sync_port=5556):
        self.context = zmq.Context()
        
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)  # Enable verbose mode
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        
        # Sync socket for acknowledgments
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        
        # Poller for subscription events
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)
        
        # Start background thread for polling subscription events
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()
        print("Background polling thread started for subscriber tracking.")

    def poll_subscriptions(self):
        """Background method to poll for subscription events."""
        while True:
            events = dict(self.poller.poll(1000))
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                message = self.pub_socket.recv_multipart()
                if message:
                    if message[0][0:1] == b'\x01':
                        Master.no_of_subscribers += 1
                        print(f"New subscriber connected. Total: {Master.no_of_subscribers}")
                    elif message[0][0:1] == b'\x00':
                        Master.pending_disconnects += 1
                        print(f"Subscriber disconnect pending: {Master.pending_disconnects}")
            time.sleep(0.1)

    def broadcast_tick(self):
        try:
            self.pub_socket.send_string("TICK")
            print("Tick broadcasted")
            
            if self.no_of_subscribers > 0:
                self.verify_acknowledgment()
            return True
        except Exception as e:
            print(f"Error broadcasting tick: {e}")
            return False
    
    def verify_acknowledgment(self):
        self.waiting_for_ack = Master.no_of_subscribers
        timeout = 10.0  # Increased timeout for slow subscribers
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        
        print(f"Waiting for {self.waiting_for_ack} acknowledgments...")
        while self.waiting_for_ack > 0 and (time.time() - start_time) < timeout:
            events = dict(poller.poll(500))  # Check every 500ms
            if self.sync_socket in events:
                try:
                    ack = self.sync_socket.recv_string()
                    self.sync_socket.send_string("ACK_RECEIVED")
                    self.waiting_for_ack -= 1
                    print(f"Acknowledgment received. Remaining: {self.waiting_for_ack}")
                except zmq.ZMQError as e:
                    print(f"Error processing acknowledgment: {e}")
                    break
            time.sleep(0.01)  # Prevent CPU overuse
        
        # Post-acknowledgment handling
        Master.no_of_subscribers = max(0, Master.no_of_subscribers - Master.pending_disconnects)
        Master.pending_disconnects = 0
        print(f"Adjusted subscribers: {Master.no_of_subscribers}")

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
    def __init__(self, sub_port=5555, sync_port=5556):
        self.context = zmq.Context()
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        self.sync_socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2-second receive timeout
        self.running = True
        self.max_messages = 10
        self.message_count = 0

    def receive_messages(self):
        print("Starting to receive messages...")
        while self.running and self.message_count < self.max_messages:
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                if message == "TICK":
                    print(f"Received message: {message}")
                    self.acknowledge_message()
                    self.message_count += 1  # Increment counter
            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in receive_messages: {e}")
                self.running = False

    def set_max_messages(self, count):
        self.max_messages = count
        
    def reset_counter(self):
        self.message_count = 0

    def acknowledge_message(self):
        print("Acknowledging message...")
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.sync_socket.send_string("ACK")
                if self.sync_socket.poll(1000, zmq.POLLIN):  # Wait up to 1 second per retry
                    reply = self.sync_socket.recv_string()
                    print(f"Received reply from server: {reply}")
                    return
                else:
                    print(f"No reply from server (attempt {attempt+1}/{max_retries}). Retrying...")
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    print(f"Timeout on attempt {attempt+1}/{max_retries}")
                else:
                    print(f"Error during acknowledgment: {e}")
                    self.reset_sync_socket()
                    break
        print("Failed to acknowledge after retries. Resetting socket...")
        self.reset_sync_socket()

    def reset_sync_socket(self):
        """Reset socket with proper cleanup and connection delay"""
        try:
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.disconnect(f"tcp://127.0.0.1:5556")
            self.sync_socket.close()
            time.sleep(0.5)  # Allow time for socket cleanup
            self.sync_socket = self.context.socket(zmq.REQ)
            self.sync_socket.setsockopt(zmq.RCVTIMEO, 2000)
            self.sync_socket.connect(f"tcp://127.0.0.1:5556")
            print("Sync socket reset successfully.")
        except Exception as e:
            print(f"Error resetting sync socket: {e}")


    # def send_termination_signal(self):
    #     try:
    #         # Send termination signal to master
    #         self.sync_socket.send_string("TERMINATE")
    #         response = self.sync_socket.recv_string()
    #         print(f"Received response from master: {response}")
    #         self.termination_event.set()  # Signal to stop receiving messages
    #         self.running = False
    #     except Exception as e:
    #         print(f"Error sending termination signal: {e}")
    # def close(self):
    #     try:
    #         self.send_termination_signal()
    #         time.sleep(1)  # Give time for the signal to be processed
    #         self.sub_socket.close()
    #         self.sync_socket.close()
    #         self.context.term()
    #         Master.no_of_subscribers -= 1
    #         print(f"Subscriber disconnected. Remaining subscribers: {Master.no_of_subscribers}")
    #     except Exception as e:
    #         print(f"Error during close: {e}")
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


