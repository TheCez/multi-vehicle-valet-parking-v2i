import json
import zmq
import time
import uuid
import carla
import threading
import numpy as np
import pickle
from occupation_grid.occupation_grid_with_grid_generator.occupation_grid_visualizer import OccupationGridVisualizer
from conflict_solver.solver import solve_conflict  # Import the conflict resolution function


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
        self.subscribers_data = {}  # Store subscriber data
        
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
            time.sleep(0.01)  # Reduced sleep for faster response

    def broadcast_tick(self):
        # Original tick broadcast
        self.pub_socket.send_string("TICK")
        print("Tick broadcasted")
        
        if Master.no_of_subscribers > 0:
            self.verify_acknowledgment()
        return True

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
                    # Receive the entire message using multipart
                    parts = self.sync_socket.recv_multipart()
                    msg_type = parts[0]
                    print(f"Received message: {msg_type}")

                    if msg_type == b"ACK":
                        print("Valid ACK received")
                        self.sync_socket.send(b"ACK_RECEIVED")
                        self.waiting_for_ack -= 1

                    elif msg_type == b"CONFLICT_DETECTION":
                        print('Conflict Checking...')
                        self.conflict_counter -= 1
                        
                        # Extract the pickled data from the multipart message
                        if len(parts) > 1:
                            subscriber_id, grid = pickle.loads(parts[1])
                            occupancy_grids[subscriber_id] = grid
                            print(f"Received occupancy grid from {subscriber_id} with shape {grid.shape}")
                            np.save(f"output_occupancy_grids/occupancy_grid_{subscriber_id}.npy", grid)
                            self.sync_socket.send(b"GRID_RECEIVED")
                            print(f"Conflict Count is {self.conflict_counter}")
                        else:
                            print("Error: CONFLICT_DETECTION message missing grid data")
                            self.sync_socket.send(b"ERROR_MISSING_DATA")
                            continue

                        if self.conflict_counter == 0:
                            # Remove subscriber_data entries whose keys are not in occupancy_grids
                            for key in list(self.subscribers_data.keys()):
                                if key not in occupancy_grids:
                                    del self.subscribers_data[key]
                                    for sub_data in self.subscribers_data.values():
                                        sub_data['car_value'] -= 2
                                        sub_data['car_reach_value'] -= 2
                            
                            # Assign unique values for '3' in each grid (starting from 4 for the second grid)
                            for idx, (subscriber_id, grid) in enumerate(occupancy_grids.items()):
                                if subscriber_id not in self.subscribers_data:
                                    new_car_value = ((Master.no_of_subscribers-1) * 2) + 2   # 4 for second, 5 for third, etc.
                                    new_car_reach_value = new_car_value + 1
                                    new_path_value = -new_car_value
                                    grid[grid == 2] = new_car_value
                                    grid[grid == 3] = new_car_reach_value
                                    grid[grid == -2] = new_path_value
                                    self.subscribers_data[subscriber_id] = {'car_value': new_car_value, 'car_reach_value': new_car_reach_value, 'path_value': new_path_value}
                                else:
                                    car_value = self.subscribers_data[subscriber_id]['car_value']
                                    car_reach_value = self.subscribers_data[subscriber_id]['car_reach_value']
                                    path_value = self.subscribers_data[subscriber_id]['path_value']
                                    grid[grid == 2] = car_value
                                    grid[grid == 3] = car_reach_value
                                    grid[grid == -2] = path_value
                            
                            # Merge all received occupancy grids with conflict handling
                            first_grid = next(iter(occupancy_grids.values()))
                            grid_shape = first_grid.shape
                            merged_grid = np.empty(grid_shape, dtype= object)
                            visualization_grid = np.empty(grid_shape, dtype=first_grid.dtype)
                            temp_grid = np.empty(grid_shape, dtype=object)
                            temp_grid_visualization = np.empty(grid_shape, dtype=first_grid.dtype)
                            visualization_grid_view = np.empty(grid_shape, dtype=first_grid.dtype)
                            top_left = None
                            top_right = None
                            bottom_left = None
                            bottom_right = None

                            # Copy the first grid as the base
                            merged_grid[:] = first_grid
                            visualization_grid[:] = first_grid
                            visualization_grid_view[:] = first_grid
                            temp_grid.fill(0)
                            temp_grid_visualization.fill(0)
                            conflict = False

                            # Merge the rest of the grids
                            for _, grid in list(occupancy_grids.items())[1:]:
                                for idx, value in np.ndenumerate(grid):
                                    if abs(merged_grid[idx])>= 2 or abs(value) >= 2:
                                        # Track the bounds of values > 3
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
                                        
                                        # If both are >= 3, add both to the list
                                        if merged_grid[idx] >= 3 and value >= 5:
                                            temp_grid[idx] = [merged_grid[idx], value]
                                            conflict = True
                                        if merged_grid[idx] >= 5 and value >= 3:
                                            temp_grid[idx] = [merged_grid[idx], value]
                                            conflict = True
                                        # If only merged_grid[idx] is >= 3, add that
                                        elif merged_grid[idx] >= 3:
                                            temp_grid[idx] = [merged_grid[idx]]
                                        # If only value is >= 3, add that
                                        elif value >= 5:
                                            temp_grid[idx] = [value]
                                        
                                        # Store the value (merged_grid[idx] or value) that has the largest absolute value
                                        if abs(visualization_grid[idx]) >= abs(value):
                                            temp_grid_visualization[idx] = visualization_grid[idx]
                                        else:
                                            temp_grid_visualization[idx] = value
                                        
                                        if visualization_grid[idx] in (2, 4) :
                                            temp_grid_visualization[idx] = visualization_grid[idx]
                                        if value in (2, 4):
                                            temp_grid_visualization[idx] = value
                                    if merged_grid[idx] != value:
                                        merged_grid[idx] = [merged_grid[idx], value]
                                        visualization_grid[idx] = max(visualization_grid[idx], value)
                                        #visualization_grid_view[idx] = max(visualization_grid_view[idx], value)
                                        # Use visualization_grid_view for value selection
                                        if visualization_grid_view[idx] == -2 or value == -2:
                                            visualization_grid_view[idx] = -2
                                        elif visualization_grid_view[idx] == -4 or value == -4:
                                            visualization_grid_view[idx] = -4
                                        elif visualization_grid_view[idx] == 5 or value == 5:
                                            visualization_grid_view[idx] = 5
                                        elif visualization_grid_view[idx] == 4 or value == 4:
                                            visualization_grid_view[idx] = 4
                                        elif visualization_grid_view[idx] == 3 or value == 3:
                                            visualization_grid_view[idx] = 3
                                        elif visualization_grid_view[idx] == 2 or value == 2:
                                            visualization_grid_view[idx] = 2
                                        elif visualization_grid_view[idx] == 1 or value == 1:
                                            visualization_grid_view[idx] = 1

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
                                conflict_area, new_path_point = solve_conflict(conflict_area)  # Call the conflict resolution function
                                self.conflict_solved = {}
                                sub_id = None
                                
                                if new_path_point is not None:
                                    if np.any((conflict_area == 4)):
                                        print("conflict_area contains 4")
                                        # Find the subscriber_id whose car_value is 4
                                        for sub_id, sub_data in self.subscribers_data.items():
                                            if sub_data['car_value'] == 4:
                                                print(f"Subscriber with car_value 4: {sub_id}")
                                                break
                                
                                for subscriber_id in occupancy_grids.keys():
                                    if subscriber_id == sub_id:
                                        conflict_area[(conflict_area == 4)] = 0
                                        self.conflict_solved[subscriber_id] = {
                                            'conflict_area': conflict_area,
                                            'new_path_point': new_path_point,
                                            'conflict_area_bounds': {
                                                'min_row': min_row,
                                                'max_row': max_row,
                                                'min_col': min_col,
                                                'max_col': max_col
                                            }
                                        }
                                    else:
                                        self.conflict_solved[subscriber_id] = 'No Conflict'
                                temp_grid_visualization = conflict_area
                            else:
                                print("No conflicts detected.")
                                self.conflict_solved = {subscriber_id: 'No Conflict' for subscriber_id in occupancy_grids.keys()}
                                
                            if len(occupancy_grids) == 1:
                                print("Only one occupancy grid received, no conflicts to resolve.")
                                self.oc.update_visualization2(current_grid=visualization_grid_view)
                            else:
                                self.oc.update_visualization2(current_grid=visualization_grid_view)
                                #self.oc.update_visualization2(current_grid=temp_grid_visualization)

                            print("Merged grid saved to output_occupancy_grids/merged_grid.npy")
                            print("All conflicts resolved! Merging occupancy grids...")
                    
                    elif msg_type == b"SEND_SOLUTION":
                        print("Received request for solution.")
                        # Send the conflict area or 'No Conflict' back to the subscriber
                        self.sync_socket.send_pyobj(self.conflict_solved)

            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    print(f"ZMQ error: {e}")
                time.sleep(0.01)

        Master.no_of_subscribers = max(0, Master.no_of_subscribers - Master.pending_disconnects)
        Master.pending_disconnects = 0
        print(f"Adjusted subscribers: {Master.no_of_subscribers}")

    def close(self):
        """Close all sockets and terminate the context."""
        try:
            self.pub_socket.setsockopt(zmq.LINGER, 0)
            self.pub_socket.close()
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.close()
            self.hb_socket.setsockopt(zmq.LINGER, 0)
            self.hb_socket.close()
            self.context.term()
            print("Master sockets closed.")
        except Exception as e:
            print(f"Error during master cleanup: {e}")


class Subscriber:
    def __init__(self, sub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        self.sync_socket.setsockopt(zmq.RCVTIMEO, 10000)  # 10-second receive timeout
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

    def send_conflict(self, occupancy_grid):
        print("Sending conflict message with retry logic...")
        max_retries = 3
        backoff = 0.1

        for attempt in range(max_retries):
            try:
                # Send multipart message: command + pickled data
                self.sync_socket.send_multipart([
                    b"CONFLICT_DETECTION",
                    pickle.dumps((self.uuid, occupancy_grid))
                ])
                print("Conflict message sent, waiting for master response...")

                if self.sync_socket.poll(5000):
                    reply = self.sync_socket.recv()
                    print(f"Received reply: {reply}")

                    if reply == b"GRID_RECEIVED":
                        print("Master acknowledged the grid.")
                        return True
                    elif reply == b"ERROR_MISSING_DATA":
                        print("Master reported missing data error, retrying...")
                        time.sleep(backoff)
                        backoff *= 2
                        continue
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
                elif isinstance(reply[self.uuid], dict):
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
            self.sync_socket.setsockopt(zmq.RCVTIMEO, 10000)
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

    try:
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
            #time.sleep(0.01)
            #master.broadcast_tick()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        master.close()
