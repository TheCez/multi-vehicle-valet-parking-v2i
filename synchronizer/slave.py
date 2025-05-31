import zmq
import json
import threading
from queue import Queue, Empty
import time
import os
from datetime import datetime
import multiprocessing

class Slave:
    def __init__(self, num_worker_threads=4):
        self.context = zmq.Context()
        self.receive_queue = Queue()
        self.num_workers = num_worker_threads
        self.workers = []
        self.running = True
        self.file_lock = threading.Lock()
        self.termination_event = threading.Event()  # Add event for termination
        
        # Create output directory if it doesn't exist
        self.output_dir = "received_data"
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = os.path.join(self.output_dir, f"Complete_diff_data.json")
        
        self.sub_socket = None
        self.sync_socket = None
        
    def receive_messages(self):
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect("tcp://127.0.0.1:5555")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect("tcp://127.0.0.1:5556")
        
        print("Sending ready signal...")
        self.sync_socket.send_string("Ready")
        response = self.sync_socket.recv_string()
        print(f"Received response: {response}")
        
        while self.running and not self.termination_event.is_set():
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                self.receive_queue.put(message)
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as e:
                print(f"Error in receive_messages: {e}")
                
    def process_message(self):
        local_sync_socket = self.context.socket(zmq.REQ)
        local_sync_socket.connect("tcp://127.0.0.1:5556")
        
        while self.running and not self.termination_event.is_set():
            try:
                message = self.receive_queue.get(timeout=1)
                data = json.loads(message)
                
                # Check for termination signal
                if isinstance(data, dict) and data.get("command") == "TERMINATE":
                    print("Received termination signal")
                    # Send final acknowledgment
                    local_sync_socket.send_string("Processed")
                    local_sync_socket.recv_string()
                    self.termination_event.set()  # Signal all threads to stop
                    self.running = False
                    break
                
                # Process regular data
                self.handle_data(data)
                
                # Send acknowledgment
                local_sync_socket.send_string("Processed")
                local_sync_socket.recv_string()
                
            except Empty:
                continue
            except Exception as e:
                print(f"Error in process_message: {e}")
        
        local_sync_socket.close()
    
    def handle_data(self, data):
        try:
            with self.file_lock:
                with open(self.output_file, 'a') as f:
                    json.dump(data, f)
                    f.write('\n')
                    f.flush()
            
            print(f"Saved geometry ID: {data.get('global_id')} to {self.output_file}")
            
        except Exception as e:
            print(f"Error saving data: {e}")
    
    def start(self):
        print(f"Starting subscriber with {self.num_workers} worker threads")
        print(f"Output file: {self.output_file}")
        
        self.receiver_thread = threading.Thread(target=self.receive_messages)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        for i in range(self.num_workers):
            worker = threading.Thread(target=self.process_message)
            worker.daemon = True
            worker.start()
            self.workers.append(worker)
            print(f"Started worker thread {i+1}")
            
    def stop(self):
        print("\nStopping subscriber...")
        self.running = False
        self.termination_event.set()  # Signal all threads to stop
        
        # Process remaining items in queue
        while not self.receive_queue.empty():
            try:
                message = self.receive_queue.get_nowait()
                data = json.loads(message)
                self.handle_data(data)
            except Empty:
                break
        
        # Close sockets
        if self.sub_socket:
            self.sub_socket.close()
        if self.sync_socket:
            self.sync_socket.close()
            
        # Wait for threads to finish
        if hasattr(self, 'receiver_thread'):
            self.receiver_thread.join(timeout=2)
        for worker in self.workers:
            worker.join(timeout=2)
            
        self.context.term()
        print("All threads stopped")

def read_saved_data(filename):
    """Utility function to verify saved data"""
    try:
        with open(filename, 'r') as f:
            for line in f:
                data = json.loads(line)
                print(f"Read geometry ID: {data.get('global_id')}")
    except Exception as e:
        print(f"Error reading file: {e}")

if __name__ == "__main__":
    try:
        subscriber = ThreadedSubscriber(num_worker_threads=multiprocessing.cpu_count())
        subscriber.start()
        
        # Keep the main thread running
        while not subscriber.termination_event.is_set():
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        subscriber.stop()
        
        # Verify saved data
        print("\nVerifying saved data:")
        read_saved_data(subscriber.output_file)
