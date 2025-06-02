import json
import zmq
import time
import uuid
import carla
import threading





class Master:
    no_of_subscribers = 0  # Static variable to keep track of subscribers
    pending_disconnects = 0  # Track pending disconnects to adjust after acknowledgment

    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)  # Enable verbose mode
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSER, 1)
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        
        # Sync socket for acknowledgments
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        
        # Poller for subscription events
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)

        # Separate heartbeat socket
        self.hb_socket = self.context.socket(zmq.ROUTER)
        self.hb_socket.bind(f"tcp://*:{hb_port}")

        # Initialize active subscribers dictionary for heartbeat tracking
        self.active_subscribers = {}

        
        # Start background thread for polling subscription events
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()
        print("Background polling thread started for subscriber tracking.")
        self.last_heartbeat = time.time()


    def poll_subscriptions(self):
        """Background method to track subscriber connections/disconnections."""
        while True:

            # Check heartbeat socket
            try:
                identity, hb = self.hb_socket.recv_multipart(zmq.NOBLOCK)
                if hb == b"HB_ACK":
                    # Update last heartbeat time for this identity
                    self.active_subscribers[identity] = time.time()
            except zmq.Again:
                pass
            events = dict(self.poller.poll(100))  # Reduced polling interval
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                while True:
                    try:
                        message = self.pub_socket.recv_multipart(zmq.NOBLOCK)
                        if message[0][0:1] == b'\x01':
                            Master.no_of_subscribers += 1
                            print(f"New subscriber. Total: {Master.no_of_subscribers}")
                        elif message[0][0:1] == b'\x00':
                            Master.no_of_subscribers = max(0, Master.no_of_subscribers - 1)
                            print(f"Subscriber disconnected. Total: {Master.no_of_subscribers}")
                    except zmq.Again:
                        break  # No more messages to process
                    #print(f"Subscriber disconnected. Total: {Master.no_of_subscribers}")
            time.sleep(0.01)  # Reduced sleep for faster response

    def broadcast_tick(self):
        try:
            # # Send heartbeat every 2 seconds
            # if time.time() - self.last_heartbeat > 2:
            #     self.pub_socket.send_string("HB")
            #     self.last_heartbeat = time.time()
                
            # Original tick broadcast
            self.pub_socket.send_string("TICK")
            print("Tick broadcasted")
            
            if Master.no_of_subscribers > 0:
                self.verify_acknowledgment()
            return True
        except Exception as e:
            print(f"Error broadcasting tick: {e}")
            return False
    
    def verify_acknowledgment(self):
        self.waiting_for_ack = Master.no_of_subscribers
        timeout = 10.0
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        
        print(f"Waiting for {self.waiting_for_ack} acks...")
        while self.waiting_for_ack > 0 and (time.time() - start_time) < timeout:
            try:
                events = dict(poller.poll(500))  # 500ms timeout per poll
                if self.sync_socket in events:
                    msg = self.sync_socket.recv()
                    if msg == b"ACK":
                        print("Valid ACK received")
                        self.sync_socket.send(b"ACK_RECEIVED")
                        self.waiting_for_ack -= 1
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    print(f"ZMQ error: {e}")
                time.sleep(0.01)
        
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
    def __init__(self, sub_port=5555, sync_port=5556, hb_port=5557):
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

        # Separate heartbeat socket
        self.hb_socket = self.context.socket(zmq.DEALER)
        self.hb_socket.connect(f"tcp://localhost:{hb_port}")

        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()


    def send_heartbeat(self):
        """Send periodic heartbeat messages to the Master to indicate liveness."""
        heartbeat_interval = 2
        while self.running:
            try:
                self.hb_socket.send(b"HB_ACK")
                time.sleep(heartbeat_interval)
            except zmq.ZMQError as e:
                print(f"Heartbeat send failed: {e}")
                time.sleep(0.3)

    def receive_messages(self):
        # last_hb = time.time()
        # hb_interval = 2  # Expected heartbeat interval
        while self.running:
                    # Send heartbeats via dedicated socket
            # if time.time() - last_hb > 2:
            #     self.hb_socket.send(b"HB_ACK")
            #     last_hb = time.time()
            try:
                # Use poll for combined message handling
                if self.sub_socket.poll(100, zmq.POLLIN):
                    message = self.sub_socket.recv_string()
                    if message == "TICK":
                        print(f"Received message: {message}")
                        self.acknowledge_message()
                        self.message_count += 1  # Increment counter
                                # Detect heartbeat timeouts
                        
                # # Graceful timeout handling
                # if time.time() - last_hb > hb_interval * 3:
                #     print("Master connection unstable...")
                #     self.reset_connection()
                #     last_hb = time.time()  # Prevent immediate retrigger
                    
            except Exception as e:
                print(f"Critical error: {e}")
                self.running = False

    def set_max_messages(self, count):
        self.max_messages = count
        
    def reset_counter(self):
        self.message_count = 0

    def acknowledge_message(self):
        print("Acknowledging message...")
        max_retries = 3
        backoff = 0.1
        
        for attempt in range(max_retries):
            try:
                self.sync_socket.send(b"ACK")
                if self.sync_socket.poll(1000):  # Wait for response
                    reply = self.sync_socket.recv()
                    print(f"Received reply: {reply.decode()}")
                    return True
            except zmq.ZMQError as e:
                print(f"Attempt {attempt+1} failed: {e}")
                time.sleep(backoff)
                backoff *= 2
        
        print("All retries failed. Attempting to reset sync socket...")
        self.reset_sync_socket()
        return False

    def reset_sync_socket(self):
        try:
            # Full cleanup sequence
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.disconnect(f"tcp://127.0.0.1:5556")
            self.sync_socket.close()
            time.sleep(0.5)  # Allow OS to release resources
            
            # Recreate context if closed
            if self.context.closed:
                self.context = zmq.Context()
                
            self.sync_socket = self.context.socket(zmq.REQ)
            self.sync_socket.setsockopt(zmq.RCVTIMEO, 2000)
            self.sync_socket.connect(f"tcp://127.0.0.1:5556")
            print("Sync socket reset successfully.")
        except Exception as e:
            print(f"Error resetting socket: {e}")
            # Emergency context recreation
            self.context = zmq.Context()
            self.sync_socket = self.context.socket(zmq.REQ)

    def close(self):
        self.running = False
        try:
            self.sub_socket.setsockopt(zmq.LINGER, 0)
            self.sub_socket.close()
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.close()
            self.hb_socket.setsockopt(zmq.LINGER, 0)
            self.hb_socket.close()
            self.context.term()
            print("Subscriber sockets closed.")
        except Exception as e:
            print(f"Error during subscriber cleanup: {e}")


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


