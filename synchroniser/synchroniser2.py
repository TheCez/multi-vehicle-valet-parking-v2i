import json
import zmq
import time
import uuid
import carla
import threading
import numpy as np
from occupation_grid.occupation_grid_with_grid_generator.occupation_grid_visualizer import OccupationGridVisualizer





class Master:
    no_of_subscribers = 0  # Static variable to keep track of subscribers
    pending_disconnects = 0  # Track pending disconnects to adjust after acknowledgment

    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        # Set up spectator (viewer) camera position and orientation
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                carla.Location(x=10.667169, y=43.477634, z=53.545383),
                carla.Rotation(pitch=-88.994576, yaw=-90.239044, roll=-0.006716)
            )
        )
        self.oc = OccupationGridVisualizer(world=world, cell_size=0.5)
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
        self.conflict_solved = None

        
        import queue
        self.visualization_queue = queue.Queue()
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
                            self.visualization_queue.put("stop")
                            if Master.no_of_subscribers == 0:
                                self.oc.stop_visualization()
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
    
    # def verify_acknowledgment(self):
    #     self.waiting_for_ack = Master.no_of_subscribers
    #     self.conflict_counter = Master.no_of_subscribers
    #     timeout = 10.0
    #     start_time = time.time()
    #     poller = zmq.Poller()
    #     poller.register(self.sync_socket, zmq.POLLIN)
        
    #     print(f"Waiting for {self.waiting_for_ack} acks...")
    #     while self.waiting_for_ack > 0 and (time.time() - start_time) < timeout:
    #         try:
    #             events = dict(poller.poll(500))  # 500ms timeout per poll
    #             if self.sync_socket in events:
    #                 msg = self.sync_socket.recv()
    #                 print(f"Received message: {msg}")
    #                 if msg == b"ACK":
    #                     print("Valid ACK received")
    #                     self.sync_socket.send(b"ACK_RECEIVED")
    #                     self.waiting_for_ack -= 1
    #                 elif msg == b"CONFLICT":
    #                     self.conflict_counter -= 1
    #                 print(f"Conflict detected! Remaining conflicts: {self.conflict_counter}")
    #                 if self.conflict_counter == 0:
    #                     print("Conflict received! Expecting two pickle files.")
    #                     self.sync_socket.send(b"SEND_GRIDS")
    #                     self.no_of_grids = Master.no_of_subscribers
    #                     occupancy_grids = []
    #                     # Receive two pickle files from the subscriber
    #                     while self.no_of_grids > 0:
    #                         occupancy_grids.append(self.sync_socket.recv_pyobj())
    #                     #occupancy_grid_2 = self.sync_socket.recv_pyobj()
    #                     # # Convert to numpy array if not already
    #                     # if not isinstance(occupancy_grid_2, np.ndarray):
    #                     #     occupancy_grid_2 = np.array(occupancy_grid_2)

    #                     # occupancy_grid_2[occupancy_grid_2 == 3] = 4
                        # # Merge the two occupancy grids, storing conflicts as lists
                        # merged_grid = np.empty_like(occupancy_grid_2, dtype=object)
                        # for idx, (val1, val2) in np.ndenumerate(zip(occupancy_grid_1.flat, occupancy_grid_2.flat)):
                        #     if val1 == val2:
                        #         merged_grid[idx] = val1
                        #     else:
                        #         merged_grid[idx] = [val1, val2]
                        # # Save the merged grid to a file
                        # np.save("merged_grid.npy", merged_grid)
    #                     # #self.solve_problem("file1.pkl", "file2.pkl")
    #                     self.sync_socket.send(b"CONFLICT_SOLVED")
    #                 else:
    #                     self.sync_socket.send(b"WAIT")
    #         except zmq.ZMQError as e:
    #             if e.errno != zmq.EAGAIN:
    #                 print(f"ZMQ error: {e}")
    #             time.sleep(0.01)
        
    #     # Post-acknowledgment handling
    #     Master.no_of_subscribers = max(0, Master.no_of_subscribers - Master.pending_disconnects)
    #     Master.pending_disconnects = 0
    #     print(f"Adjusted subscribers: {Master.no_of_subscribers}")


    def verify_acknowledgment(self):
        self.waiting_for_ack = Master.no_of_subscribers
        self.conflict_counter = Master.no_of_subscribers
        timeout = 10.0
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        # Create a list to hold occupancy grids, one for each subscriber
        occupancy_grids = {}

        print(f"Waiting for {self.waiting_for_ack} acks or conflicts...")

        while (self.waiting_for_ack > 0 or self.conflict_counter > 0) and (time.time() - start_time) < timeout:
            try:
                events = dict(poller.poll(500))
                if self.sync_socket in events:
                    msg = self.sync_socket.recv()
                    print(f"Received message: {msg}")

                    if msg == b"ACK":
                        print("Valid ACK received")
                        self.sync_socket.send(b"ACK_RECEIVED")
                        self.waiting_for_ack -= 1

                    elif msg == b"CONFLICT_DETECTION":
                        print('Conflict Checking...')
                        self.conflict_counter -= 1
                        self.sync_socket.send(b"SEND_GRID")  # Respond immediately

                        # Now receive the grid from this subscriber
                        subscriber_id, grid = self.sync_socket.recv_pyobj()

                        occupancy_grids[subscriber_id] = grid  # Store the grid with subscriber ID as key
                        # Find the first available slot (None) and store the grid there
                        # for idx in range(len(occupancy_grids)):
                        #     if occupancy_grids[idx] is None:
                        #         occupancy_grids[idx] = grid
                        #         break
                        print(f"Received occupancy grid from {subscriber_id} with shape {grid.shape}")
                        np.save(f"output_occupancy_grids/occupancy_grid_{subscriber_id}.npy", grid)
                        self.sync_socket.send(b"GRID_RECEIVED")  # Acknowledge grid receipt
                        print(f"Conflict Count is {self.conflict_counter}")

                        if self.conflict_counter == 0:
                            # Assign unique values for '3' in each grid (starting from 4 for the second grid)
                            for idx, (subscriber_id, grid) in enumerate(occupancy_grids.items()):
                                if idx == 0:
                                    continue  # Skip the first grid
                                new_value = 3 + idx  # 4 for second, 5 for third, etc.
                                grid[grid == 3] = new_value
                            
                            # Merge all received occupancy grids with conflict handling
                            first_grid = next(iter(occupancy_grids.values()))
                            grid_shape = first_grid.shape
                            merged_grid = np.empty(grid_shape, dtype= object)
                            visualization_grid = np.empty(grid_shape, dtype=first_grid.dtype)
                            temp_grid = np.empty(grid_shape, dtype=object)
                            temp_grid_visualization = np.empty(grid_shape, dtype=first_grid.dtype)
                            top_left = None
                            top_right = None
                            bottom_left = None
                            bottom_right = None
                            


                            # Copy the first grid as the base
                            merged_grid[:] = first_grid
                            visualization_grid[:] = first_grid
                            temp_grid.fill(1)
                            temp_grid_visualization.fill(1)
                            conflict = False

                            # Merge the rest of the grids
                            for _, grid in list(occupancy_grids.items())[1:]:
                                for idx, value in np.ndenumerate(grid):
                                    #print('Here')
                                    #print(idx, value)
                                    if merged_grid[idx]>= 2 or value >= 2:
                                        # Track the bounds of values > 3
                                        #if value > 3 or merged_grid[idx] > 3:
                                        row, col = idx
                                        if top_left is None:
                                            top_left = (row, col)
                                            bottom_right = (row, col)
                                            top_right = (row, col)
                                            bottom_left = (row, col)
                                        else:
                                            # Update bounds
                                            if row < top_left[0] or col < top_left[1]:
                                                top_left = (min(row, top_left[0]), min(col, top_left[1]))
                                            if row < bottom_left[0] or col > bottom_left[1]:
                                                bottom_left = (min(row, bottom_left[0]), max(col, bottom_left[1]))
                                            if row > top_right[0] or col < top_right[1]:
                                                top_right = (max(row, top_right[0]), min(col, top_right[1]))
                                            if row > bottom_right[0] or col > bottom_right[1]:
                                                bottom_right = (max(row, bottom_right[0]), max(col, bottom_right[1]))
                                        #print('Here!!!!')
                                        # If both are >= 3, add both to the list
                                        if merged_grid[idx] >= 2 and value >= 2:
                                            temp_grid[idx] = [merged_grid[idx], value]
                                            conflict = True
                                        # If only merged_grid[idx] is >= 3, add that
                                        elif merged_grid[idx] >= 2:
                                            temp_grid[idx] = [merged_grid[idx]]
                                        # If only value is >= 3, add that
                                        elif value >= 2:
                                            temp_grid[idx] = [value]
                                        temp_grid_visualization[idx] = min(visualization_grid[idx], value)
                                    if merged_grid[idx] != value:
                                        merged_grid[idx] = [merged_grid[idx], value]

                                        visualization_grid[idx] = max(visualization_grid[idx], value)
                                    # else: values are the same, do nothing
                            # If there was a conflict
                            if conflict:
                                print("Conflict detected!")
                                # Ensure all corner points are set
                                if None not in (top_left, top_right, bottom_left, bottom_right):
                                    # Find min/max rows and cols to define the bounding box
                                    min_row = min(top_left[0], bottom_left[0])
                                    max_row = max(top_right[0], bottom_right[0])
                                    min_col = min(top_left[1], top_right[1])
                                    max_col = max(bottom_left[1], bottom_right[1])

                                    # Extract the subgrid containing all four points
                                    conflict_area = temp_grid_visualization[min_row:max_row+1, min_col:max_col+1]
                                    print(f"Extracted conflict area shape: {conflict_area.shape}")
                                    np.save("output_occupancy_grids/conflict_area.npy", conflict_area)
                                else:
                                    print("Could not determine all four corners for conflict area extraction.")
                                # Store the conflict area for each subscriber
                                self.conflict_solved = {}
                                for subscriber_id in occupancy_grids.keys():
                                    self.conflict_solved[subscriber_id] = conflict_area
                                
                            else:
                                print("No conflicts detected.")
                                self.conflict_solved = {subscriber_id: 'No Conflict' for subscriber_id in occupancy_grids.keys()}
                                # Save the merged grid to a file
                                
                            if len(occupancy_grids) == 1:
                                print("Only one occupancy grid received, no conflicts to resolve.")
                                self.oc.update_visualization2(current_grid=visualization_grid)
                            else:
                                self.oc.update_visualization2(current_grid=temp_grid_visualization)
                            #self.oc.update_visualization2(current_grid = visualization_grid)

                            #np.save("output_occupancy_grids/merged_grid.npy", merged_grid)
                            print("Merged grid saved to output_occupancy_grids/merged_grid.npy")
                            print("All conflicts resolved! Merging occupancy grids...")
                    elif msg == b"SEND_SOLUTION":
                        print("Received request for solution.")
                        # Send the conflict area or 'No Conflict' back to the subscriber
                        
                        self.sync_socket.send_pyobj(self.conflict_solved)
                        #reply = self.sync_socket.recv()
                        #if reply == b"SOLUTION_RECEIVED":
                        #    print("Solution acknowledged by subscriber.")

            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    print(f"ZMQ error: {e}")
                time.sleep(0.01)

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
        self.sync_socket.setsockopt(zmq.RCVTIMEO, 10000)  # 2-second receive timeout
        self.running = True
        self.uuid = str(uuid.uuid4())  # Unique identifier for this subscriber

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
        """Check for a single 'TICK' message and return True if received."""
        if not self.running:
            return False

        try:
            # Use poll for message handling with a short timeout
            if self.sub_socket.poll(100, zmq.POLLIN):
                message = self.sub_socket.recv_string()
                if message == "TICK":
                    print(f"Received message: {message}")
                    return True
        except Exception as e:
            print(f"Critical error: {e}")
            self.running = False
        return False

    def acknowledge_message(self):
        """Acknowledge a received message with retry logic."""
        print("Acknowledging message...")
        max_retries = 3
        backoff = 0.1
        
        for attempt in range(max_retries):
            try:
                self.sync_socket.send(b"ACK")
                if self.sync_socket.poll(10000):  # Wait for response
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
    

    # def send_conflict(self, occupancy_grid):

    #     #try:
    #         # Send 'CONFLICT' message
    #         self.sync_socket.send(b"CONFLICT")
    #         print("Conflict message sent, waiting for master response...")
    #         # Wait for master to be ready (optional, depending on your protocol)
    #         # Send two pickle files as bytes
    #         reply = self.sync_socket.recv()
    #         print(f"Received reply: {reply}")
    #         if reply == b"SEND_GRIDS":
    #             self.sync_socket.send_pyobj(occupancy_grid)
    #         # Wait for response
    #         reply = self.sync_socket.recv()
    #         if reply == b"CONFLICT_SOLVED":
    #             print("Master solved the problem with the provided pickle files.")
    #     # except Exception as e:
    #     #     print(f"Error sending conflict: {e}")
    #     #     self.reset_sync_socket()
    
    def send_conflict(self, occupancy_grid):
        print("Sending conflict message with retry logic...")
        max_retries = 3
        backoff = 0.1

        for attempt in range(max_retries):
            try:
                self.sync_socket.send(b"CONFLICT_DETECTION")
                print("Conflict message sent, waiting for master response...")

                if self.sync_socket.poll(5000):
                    reply = self.sync_socket.recv()
                    print(f"Received reply: {reply}")

                    if reply == b"SEND_GRID":
                        self.sync_socket.send_pyobj((self.uuid,occupancy_grid))

                        if self.sync_socket.poll(5000):
                            reply = self.sync_socket.recv()
                            print(f"Received reply: {reply}")
                            if reply == b"GRID_RECEIVED":
                                print("Master acknowledged the grid.")
                                return True
                            else:
                                print("Unexpected reply after sending grid, retrying...")
                        else:
                            print("No response after sending occupancy grid, retrying...")
                    elif reply == b"WAIT":
                        print("Master not ready, will retry after backoff...")
                        time.sleep(backoff)
                        backoff *= 2
                        continue  # Retry protocol
                    else:
                        print("Unexpected reply from master, retrying...")
                else:
                    print("No response from master, retrying...")
            except zmq.ZMQError as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                time.sleep(backoff)
                backoff *= 2
                self.reset_sync_socket()
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                time.sleep(backoff)
                backoff *= 2
                self.reset_sync_socket()

        print("All retries failed. Could not send conflict message.")
        return False
    
    def receive_solution(self):
        """Receive the conflict area or 'No Conflict' from the master."""
        try:
            self.sync_socket.send(b"SEND_SOLUTION")
            if self.sync_socket.poll(5000):
                reply = self.sync_socket.recv_pyobj()
                if reply is None:
                    print("No Conflict")
                    return None
                if isinstance(reply[self.uuid], str) and reply[self.uuid] == 'No Conflict':
                    print("Received reply: No Conflict")
                    return None  # No conflict area to return
                elif isinstance(reply[self.uuid], np.ndarray):
                    print("Received reply: Conflict Area")
                    return reply[self.uuid]  # Return the conflict area for this subscriber
                else:
                    print("Unexpected reply format.")
            else:
                print("No response from master.")
        except Exception as e:
            print(f"Error receiving solution: {e}")
            self.reset_sync_socket()
        return None   

    def reset_sync_socket(self):
        """Reset the sync socket in case of failure."""
        try:
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.disconnect(f"tcp://127.0.0.1:5556")
            self.sync_socket.close()
            time.sleep(0.5)  # Allow OS to release resources
            
            if self.context.closed:
                self.context = zmq.Context()
                
            self.sync_socket = self.context.socket(zmq.REQ)
            self.sync_socket.setsockopt(zmq.RCVTIMEO, 2000)
            self.sync_socket.connect(f"tcp://127.0.0.1:5556")
            print("Sync socket reset successfully.")
        except Exception as e:
            print(f"Error resetting socket: {e}")
            self.context = zmq.Context()
            self.sync_socket = self.context.socket(zmq.REQ)

    def close(self):
        """Close all sockets and terminate the context."""
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

    while True:
        world.tick()
        print("CARLA world ticked")
        print(master.no_of_subscribers)
        master.broadcast_tick()
        # Check for visualization commands from the background thread
        try:
            while not master.visualization_queue.empty():
                cmd = master.visualization_queue.get_nowait()
                if cmd == "stop":
                    master.oc.stop_visualization()
        except Exception as e:
            print(f"Error handling visualization command: {e}")
        time.sleep(0.1)
        master.broadcast_tick()
        time.sleep(0.1)
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


