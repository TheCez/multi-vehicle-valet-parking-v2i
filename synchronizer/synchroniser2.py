import json
import zmq
import time
import uuid
import carla
import threading
from threading import Lock





class Master:
    no_of_subscribers = 0  # Static variable to keep track of subscribers
    pending_disconnects = 0  # Track pending disconnects to adjust after acknowledgment

    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        self.context.set(zmq.IO_THREADS, 1)  # Reduce to 1 to minimize background thread conflicts
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        self.pub_socket.setsockopt(zmq.SNDHWM, 100)
        self.pub_socket.setsockopt(zmq.IMMEDIATE, 1)
        try:
            self.pub_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.sync_socket = self.context.socket(zmq.ROUTER)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        self.sync_socket.setsockopt(zmq.HEARTBEAT_IVL, 2000)
        self.sync_socket.setsockopt(zmq.HEARTBEAT_TIMEOUT, 5000)
        self.sync_socket.setsockopt(zmq.HEARTBEAT_TTL, 5000)
        self.sync_socket.setsockopt(zmq.SNDHWM, 100)
        self.sync_socket.setsockopt(zmq.RCVHWM, 100)
        try:
            self.sync_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.hb_socket = self.context.socket(zmq.ROUTER)
        self.hb_socket.bind(f"tcp://127.0.0.1:{hb_port}")
        self.hb_socket.setsockopt(zmq.HEARTBEAT_IVL, 2000)
        self.hb_socket.setsockopt(zmq.HEARTBEAT_TIMEOUT, 5000)
        self.hb_socket.setsockopt(zmq.HEARTBEAT_TTL, 5000)
        self.hb_socket.setsockopt(zmq.SNDHWM, 100)
        self.hb_socket.setsockopt(zmq.RCVHWM, 100)
        try:
            self.hb_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)
        self.active_subscribers = {}
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()
        print("Background polling thread started for subscriber tracking.")
        self.last_heartbeat = time.time()


    def poll_subscriptions(self):
        """Background method to track subscriber connections/disconnections."""
        heartbeat_timeout = 10  # Increased from 6 to 8 seconds to avoid premature timeouts
        while True:
            try:
                msg_parts = self.hb_socket.recv_multipart(zmq.NOBLOCK)
                if len(msg_parts) >= 2:
                    identity = msg_parts[0]
                    hb = msg_parts[1]
                    if hb == b"HB_ACK":
                        if identity not in self.active_subscribers and Master.no_of_subscribers > len(self.active_subscribers):
                            print(f"New subscriber added via heartbeat. ID: {identity}, Total: {Master.no_of_subscribers}")
                        self.active_subscribers[identity] = time.time()
                        print(f"Heartbeat received from subscriber {identity}")
            except zmq.Again:
                pass

            events = dict(self.poller.poll(20))  # Reduced from 100ms to 50ms for faster response
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                message = self.pub_socket.recv_multipart()
                if message[0][0:1] == b'\x01':
                    Master.no_of_subscribers += 1
                    print(f"New subscriber detected. Total: {Master.no_of_subscribers}")
                elif message[0][0:1] == b'\x00':
                    Master.pending_disconnects += 1
                    print(f"Subscriber disconnected. Pending disconnects: {Master.pending_disconnects}")

            current_time = time.time()
            inactive_subscribers = [
                identity for identity, last_hb in self.active_subscribers.items()
                if current_time - last_hb > heartbeat_timeout
            ]
            for identity in inactive_subscribers:
                del self.active_subscribers[identity]
                Master.pending_disconnects += 1
                print(f"Subscriber {identity} timed out. Pending disconnects: {Master.pending_disconnects}")

            time.sleep(0.002)  # Reduced from 0.01 to 0.005 for faster polling


    def broadcast_tick(self):
        try:
            self.pub_socket.send_string("TICK")
            print("Tick broadcasted")
            if Master.no_of_subscribers > 0:
                self.verify_acknowledgment(max_wait=0.015)  # 15ms max wait, less than half of 33.3ms for 30 FPS
            return True
        except Exception as e:
            print(f"Error broadcasting tick: {e}")
            return False

    
    def verify_acknowledgment(self, max_wait=0.015):
        self.waiting_for_ack = Master.no_of_subscribers
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        received_acks = set()
        
        print(f"Waiting for {self.waiting_for_ack} acks...")
        while (self.waiting_for_ack > 0 or len(self.active_subscribers) > 0) and (time.time() - start_time) < max_wait:
            try:
                events = dict(poller.poll(5))  # Reduced from 50ms to 5ms per poll for faster response
                if self.sync_socket in events:
                    msg_parts = self.sync_socket.recv_multipart()
                    if len(msg_parts) >= 2:
                        identity = msg_parts[0]
                        msg = msg_parts[1]
                        if msg == b"ACK" and identity not in received_acks:
                            print(f"Valid ACK received from {identity}")
                            self.sync_socket.send_multipart([identity, b"ACK_RECEIVED"])
                            received_acks.add(identity)
                            if self.waiting_for_ack > 0:
                                self.waiting_for_ack -= 1
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    print(f"ZMQ error: {e}")
                time.sleep(0.0001)  # Minimal sleep for faster response
        
        Master.no_of_subscribers = max(0, Master.no_of_subscribers - Master.pending_disconnects)
        Master.pending_disconnects = 0
        active_count = len(self.active_subscribers)
        if active_count != Master.no_of_subscribers:
            print(f"Adjusting subscriber count to match active subscribers: {active_count}")
            Master.no_of_subscribers = active_count
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

import zmq
import threading
import time
import uuid
from threading import Lock
from queue import Queue, Empty

class Subscriber:
    def __init__(self, sub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        self.context.set(zmq.IO_THREADS, 1)  # Reduce to 1 to minimize background thread conflicts
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"TICK")
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt(zmq.RCVHWM, 100)
        try:
            self.sub_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.sync_socket = self.context.socket(zmq.DEALER)
        self.identity = str(uuid.uuid4()).encode()
        self.sync_socket.setsockopt(zmq.IDENTITY, self.identity)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        self.sync_socket.setsockopt(zmq.HEARTBEAT_IVL, 2000)
        self.sync_socket.setsockopt(zmq.HEARTBEAT_TIMEOUT, 5000)
        self.sync_socket.setsockopt(zmq.HEARTBEAT_TTL, 5000)
        self.sync_socket.setsockopt(zmq.SNDHWM, 100)
        self.sync_socket.setsockopt(zmq.RCVHWM, 100)
        try:
            self.sync_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.hb_socket = self.context.socket(zmq.DEALER)
        self.hb_socket.setsockopt(zmq.IDENTITY, self.identity)
        self.hb_socket.connect(f"tcp://127.0.0.1:{hb_port}")
        self.hb_socket.setsockopt(zmq.HEARTBEAT_IVL, 2000)
        self.hb_socket.setsockopt(zmq.HEARTBEAT_TIMEOUT, 5000)
        self.hb_socket.setsockopt(zmq.HEARTBEAT_TTL, 5000)
        self.hb_socket.setsockopt(zmq.SNDHWM, 100)
        try:
            self.hb_socket.setsockopt(zmq.TCP_NODELAY, 1)
        except AttributeError:
            print("TCP_NODELAY not supported on this platform")
        
        self.running = True
        self.ack_queue = Queue()  # Queue for acknowledgment requests
        self.ack_result_queue = Queue()  # Queue for acknowledgment results
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()
        self.receive_thread = threading.Thread(target=self.receive_messages, daemon=True)
        self.receive_thread.start()
        self.ack_thread = threading.Thread(target=self.handle_acknowledgments, daemon=True)
        self.ack_thread.start()  # Dedicated thread for acknowledgment handling

    def send_heartbeat(self):
        """Send periodic heartbeat messages to the Master to indicate liveness."""
        heartbeat_interval = 6
        while self.running:
            try:
                self.hb_socket.send(b"HB_ACK")
                time.sleep(heartbeat_interval)
            except zmq.ZMQError as e:
                print(f"Heartbeat send failed: {e}")
                time.sleep(0.3)

    def receive_messages(self):
        while self.running:
            try:
                if self.sub_socket.poll(10, zmq.POLLIN):
                    message = self.sub_socket.recv_string()
                    if message == "TICK":
                        print(f"Received message: {message}")
                        self.ack_queue.put(True)  # Request acknowledgment via queue
                        try:
                            # Wait briefly for acknowledgment result
                            result = self.ack_result_queue.get(timeout=0.05)
                            if not result:
                                print("Acknowledgment failed or timed out.")
                        except Empty:
                            print("No acknowledgment result received in time.")
            except Exception as e:
                print(f"Critical error in receive_messages: {e}")
                self.running = False

    def handle_acknowledgments(self):
        """Dedicated thread to handle acknowledgment send/receive operations."""
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        while self.running:
            try:
                # Check for acknowledgment requests
                try:
                    self.ack_queue.get_nowait()  # Non-blocking check for request
                    print("Acknowledging message...")
                    request_timeout = 50
                    max_retries = 3
                    retries_left = max_retries
                    success = False
                    while retries_left > 0 and self.running and not success:
                        try:
                            self.sync_socket.send(b"ACK")
                            socks = dict(poller.poll(request_timeout))
                            if socks.get(self.sync_socket) == zmq.POLLIN:
                                reply = self.sync_socket.recv()
                                print(f"Received reply: {reply.decode()}")
                                success = True
                                self.ack_result_queue.put(True)
                            else:
                                print(f"No response from Master, retrying... ({retries_left} retries left)")
                                retries_left -= 1
                                if retries_left == 0:
                                    print("All retries failed. Continuing with current socket.")
                                    self.ack_result_queue.put(False)
                                time.sleep(0.01)
                        except zmq.ZMQError as e:
                            print(f"Attempt failed: {e}")
                            retries_left -= 1
                            if retries_left == 0:
                                print("All retries failed. Continuing with current socket.")
                                self.ack_result_queue.put(False)
                            time.sleep(0.01)
                except Empty:
                    time.sleep(0.001)  # No request, brief sleep to avoid CPU overuse
            except Exception as e:
                print(f"Critical error in handle_acknowledgments: {e}")
                self.ack_result_queue.put(False)
                time.sleep(0.01)  # Brief delay before retrying

    def close(self):
        self.running = False
        try:
            # Wait for threads to finish current operations with a longer timeout
            self.heartbeat_thread.join(timeout=2.0)
            self.receive_thread.join(timeout=2.0)
            self.ack_thread.join(timeout=2.0)
            # Clear any pending items in queues
            while not self.ack_queue.empty():
                try:
                    self.ack_queue.get_nowait()
                except Empty:
                    break
            while not self.ack_result_queue.empty():
                try:
                    self.ack_result_queue.get_nowait()
                except Empty:
                    break
            # Close sockets with LINGER set to 0 to discard pending messages
            self.sub_socket.setsockopt(zmq.LINGER, 0)
            self.sub_socket.close()
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.close()
            self.hb_socket.setsockopt(zmq.LINGER, 0)
            self.hb_socket.close()
            time.sleep(0.2)  # Increased delay to ensure all operations complete
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


