import json
import zmq
import time
import uuid
import carla
import threading
import numpy as np
from occupation_grid.occupation_grid_with_grid_generator.occupation_grid_visualizer import OccupationGridVisualizer

class Master:
    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                carla.Location(x=10.667169, y=43.477634, z=53.545383),
                carla.Rotation(pitch=-88.994576, yaw=-90.239044, roll=-0.006716)
            )
        )
        self.oc = OccupationGridVisualizer(world=world, cell_size=0.5)
        self.context = zmq.Context()
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSER, 1)
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)
        self.hb_socket = self.context.socket(zmq.ROUTER)
        self.hb_socket.bind(f"tcp://*:{hb_port}")

        self.active_subscribers = {}  # identity -> last_hb_time

        import queue
        self.visualization_queue = queue.Queue()
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()
        print("Background polling thread started for subscriber tracking.")
        self.last_heartbeat = time.time()

    def poll_subscriptions(self):
        while True:
            try:
                identity, hb = self.hb_socket.recv_multipart(zmq.NOBLOCK)
                if hb == b"HB_ACK":
                    self.active_subscribers[identity] = time.time()
            except zmq.Again:
                pass
            events = dict(self.poller.poll(100))
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                while True:
                    try:
                        message = self.pub_socket.recv_multipart(zmq.NOBLOCK)
                        if message[0][0:1] == b'\x01':
                            print(f"New subscriber. Total: {len(self.active_subscribers)}")
                        elif message[0][0:1] == b'\x00':
                            print(f"Subscriber disconnected. Total: {len(self.active_subscribers)}")
                            self.visualization_queue.put("stop")
                            if not self.active_subscribers:
                                self.oc.stop_visualization()
                    except zmq.Again:
                        break
            time.sleep(0.01)

    def broadcast_tick(self):
        self.tick_subscriber_set = set(self.active_subscribers.keys())
        self.tick_subscriber_count = len(self.tick_subscriber_set)
        self.pub_socket.send_string("TICK")
        print("Tick broadcasted")
        if self.tick_subscriber_count > 0:
            self.verify_acknowledgment()

    def verify_acknowledgment(self):
        waiting_for_ack = self.tick_subscriber_count
        conflict_counter = self.tick_subscriber_count
        solution_counter = self.tick_subscriber_count
        timeout = 10.0
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        occupancy_grids = {}
        grid_contributors = set()
        print(f"Waiting for {waiting_for_ack} acks or conflicts from {self.tick_subscriber_count} subscribers ...")

        while (waiting_for_ack > 0 or conflict_counter > 0) and (time.time() - start_time) < timeout:
            try:
                events = dict(poller.poll(500))
                if self.sync_socket in events:
                    msg = self.sync_socket.recv()
                    print(f"Received message: {msg}")

                    if msg == b"ACK":
                        print("Valid ACK received")
                        self.sync_socket.send(b"ACK_RECEIVED")
                        waiting_for_ack -= 1

                    elif msg == b"CONFLICT_DETECTION":
                        print('Conflict Checking...')
                        self.sync_socket.send(b"SEND_GRID")
                        subscriber_id, grid = self.sync_socket.recv_pyobj()
                        occupancy_grids[subscriber_id] = grid
                        grid_contributors.add(subscriber_id)
                        self.sync_socket.send(b"GRID_RECEIVED")
                        conflict_counter -= 1
                        print(f"Conflict Count is {conflict_counter}")

                        if conflict_counter == 0:
                            for idx, (sub_id, grid) in enumerate(occupancy_grids.items()):
                                if idx == 0:
                                    continue
                                new_val = 3 + idx
                                grid[grid == 3] = new_val
                            first_grid = next(iter(occupancy_grids.values()))
                            grid_shape = first_grid.shape
                            merged_grid = np.empty(grid_shape, dtype=object)
                            visualization_grid = np.empty(grid_shape, dtype=first_grid.dtype)
                            temp_grid = np.empty(grid_shape, dtype=object)
                            temp_grid_visualization = np.empty(grid_shape, dtype=first_grid.dtype)
                            merged_grid[:] = first_grid
                            visualization_grid[:] = first_grid
                            temp_grid.fill(1)
                            temp_grid_visualization.fill(1)
                            conflict = False
                            top_left = top_right = bottom_left = bottom_right = None
                            for _, grid in list(occupancy_grids.items())[1:]:
                                for idx, value in np.ndenumerate(grid):
                                    if merged_grid[idx] >= 2 or value >= 2:
                                        row, col = idx
                                        if top_left is None:
                                            top_left = (row, col)
                                            bottom_right = (row, col)
                                            top_right = (row, col)
                                            bottom_left = (row, col)
                                        else:
                                            if row < top_left[0] or col < top_left[1]:
                                                top_left = (min(row, top_left[0]), min(col, top_left[1]))
                                            if row < bottom_left[0] or col > bottom_left[1]:
                                                bottom_left = (min(row, bottom_left[0]), max(col, bottom_left[1]))
                                            if row > top_right[0] or col < top_right[1]:
                                                top_right = (max(row, top_right[0]), min(col, top_right[1]))
                                            if row > bottom_right[0] or col > bottom_right[1]:
                                                bottom_right = (max(row, bottom_right[0]), max(col, bottom_right[1]))
                                        if merged_grid[idx] >= 2 and value >= 2:
                                            temp_grid[idx] = [merged_grid[idx], value]
                                            conflict = True
                                        elif merged_grid[idx] >= 2:
                                            temp_grid[idx] = [merged_grid[idx]]
                                        elif value >= 2:
                                            temp_grid[idx] = [value]
                                        temp_grid_visualization[idx] = min(visualization_grid[idx], value)
                                    if merged_grid[idx] != value:
                                        merged_grid[idx] = [merged_grid[idx], value]
                                        visualization_grid[idx] = max(visualization_grid[idx], value)
                            if conflict:
                                print("Conflict detected!")
                                if None not in (top_left, top_right, bottom_left, bottom_right):
                                    min_row = min(top_left[0], bottom_left[0])
                                    max_row = max(top_right[0], bottom_right[0])
                                    min_col = min(top_left[1], top_right[1])
                                    max_col = max(bottom_left[1], bottom_right[1])
                                    conflict_area = temp_grid_visualization[min_row:max_row + 1, min_col:max_col + 1]
                                    print(f"Extracted conflict area shape: {conflict_area.shape}")
                                    np.save("output_occupancy_grids/conflict_area.npy", conflict_area)
                                else:
                                    print("Could not determine all four corners for conflict area extraction.")
                                conflict_solved = {sub_id: conflict_area for sub_id in occupancy_grids.keys()}
                            else:
                                print("No conflicts detected.")
                                conflict_solved = {sub_id: 'No Conflict' for sub_id in occupancy_grids.keys()}
                            for _ in grid_contributors:
                                status = self.sync_socket.recv()
                                if status == b"SEND_SOLUTION":
                                    self.sync_socket.send_pyobj(conflict_solved)
                            if len(occupancy_grids) == 1:
                                print("Only one occupancy grid received, no conflicts to resolve.")
                                self.oc.update_visualization2(current_grid=visualization_grid)
                            else:
                                self.oc.update_visualization2(current_grid=temp_grid_visualization)
                            print("Merged grid saved to output_occupancy_grids/merged_grid.npy")
                            print("All conflicts resolved! Merging occupancy grids...")
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    print(f"ZMQ error: {e}")
                time.sleep(0.01)
        print("Finished ack/conflict handling for this tick.")

class Subscriber:
    def __init__(self, sub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        self.sync_socket.setsockopt(zmq.RCVTIMEO, 10000) 
        self.running = True
        self.uuid = str(uuid.uuid4())
        self.hb_socket = self.context.socket(zmq.DEALER)
        self.hb_socket.connect(f"tcp://localhost:{hb_port}")
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def send_heartbeat(self):
        heartbeat_interval = 2
        while self.running:
            try:
                self.hb_socket.send(b"HB_ACK")
                time.sleep(heartbeat_interval)
            except zmq.ZMQError as e:
                print(f"Heartbeat send failed: {e}")
                time.sleep(0.3)

    def receive_messages(self):
        """Waits for a TICK, returns True if received."""
        if not self.running:
            return False
        try:
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
        print("Acknowledging message...")
        max_retries = 3
        backoff = 0.1
        for attempt in range(max_retries):
            try:
                self.sync_socket.send(b"ACK")
                if self.sync_socket.poll(10000):
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
                self.sync_socket.send(b"CONFLICT_DETECTION")
                print("Conflict message sent, waiting for master response...")
                if self.sync_socket.poll(5000):
                    reply = self.sync_socket.recv()
                    print(f"Received reply: {reply}")
                    if reply == b"SEND_GRID":
                        self.sync_socket.send_pyobj((self.uuid, occupancy_grid))
                        if self.sync_socket.poll(5000):
                            reply = self.sync_socket.recv()
                            if reply == b"GRID_RECEIVED":
                                print("Master acknowledged the grid.")
                                self.sync_socket.send(b"SEND_SOLUTION")
                                if self.sync_socket.poll(5000):
                                    reply = self.sync_socket.recv_pyobj()
                                    if reply[self.uuid] == 'No Conflict':
                                        print("Received reply: No Conflict")
                                        return True
                                    elif isinstance(reply[self.uuid], np.ndarray):
                                        print('Received reply: Conflict Area')
                                        return reply[self.uuid]
                            else:
                                print("Unexpected reply after sending grid, retrying...")
                        else:
                            print("No response after sending occupancy grid, retrying...")
                    elif reply == b"WAIT":
                        print("Master not ready, will retry after backoff...")
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

    def reset_sync_socket(self):
        try:
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.disconnect(f"tcp://127.0.0.1:5556")
            self.sync_socket.close()
            time.sleep(0.5)
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

    def run_protocol(self, get_occupancy_grid_func):
        """Call this in your car logic main loop.
        Only participate after receiving TICK."""
        while self.running:
            tick_received = self.receive_messages()
            if tick_received:
                ack_ok = self.acknowledge_message()
                if ack_ok:
                    grid = get_occupancy_grid_func()  # supply your occupancy_grid builder per tick
                    self.send_conflict(grid)
            else:
                # Don't do anything until a TICK is seen
                time.sleep(0.05)

# Example usage for master:
if __name__ == "__main__":
    master = Master()
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    client.set_timeout(10.0)
    world = client.get_world()
    while True:
        world.tick()
        print("CARLA world ticked")
        print(len(master.active_subscribers))
        master.broadcast_tick()
        try:
            while not master.visualization_queue.empty():
                cmd = master.visualization_queue.get_nowait()
                if cmd == "stop":
                    master.oc.stop_visualization()
        except Exception as e:
            print(f"Error handling visualization command: {e}")
        time.sleep(0.1)
