import json
import zmq
import time
import uuid
import carla
import threading
import numpy as np
import pickle
from occupation_grid.occupation_grid_with_grid_generator.occupation_grid_visualizer import OccupationGridVisualizer
from conflict_solver.solver2 import solve_conflict
import os


class Master:
    no_of_subscribers = 0
    pending_disconnects = 0

    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557):
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                carla.Location(x=10.667169, y=43.477634, z=43.545383),
                carla.Rotation(pitch=-88.994576, yaw=-90.239044, roll=-0.006716)
            )
        )
        
        self.oc = OccupationGridVisualizer(world=world, cell_size=0.5)
        self.context = zmq.Context()
        
        # Optimized Publisher socket
        self.pub_socket = self.context.socket(zmq.XPUB)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSE, 1)
        self.pub_socket.setsockopt(zmq.XPUB_VERBOSER, 1)
        self.pub_socket.setsockopt(zmq.SNDHWM, 10000)
        self.pub_socket.setsockopt(zmq.RCVHWM, 10000)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.pub_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.pub_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.pub_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.pub_socket.bind(f"tcp://127.0.0.1:{pub_port}")
        
        # Optimized Sync socket
        self.sync_socket = self.context.socket(zmq.REP)
        self.sync_socket.setsockopt(zmq.SNDHWM, 10000)
        self.sync_socket.setsockopt(zmq.RCVHWM, 10000)
        self.sync_socket.setsockopt(zmq.LINGER, 0)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.sync_socket.bind(f"tcp://127.0.0.1:{sync_port}")
        
        # Optimized Heartbeat socket
        self.hb_socket = self.context.socket(zmq.ROUTER)
        self.hb_socket.setsockopt(zmq.SNDHWM, 10000)
        self.hb_socket.setsockopt(zmq.RCVHWM, 10000)
        self.hb_socket.setsockopt(zmq.LINGER, 0)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.hb_socket.bind(f"tcp://*:{hb_port}")
        
        self.poller = zmq.Poller()
        self.poller.register(self.pub_socket, zmq.POLLIN)
        
        self.active_subscribers = {}
        self.conflict_solved = None
        self.subscribers_data = {}
        
        import queue

        self.visualization_queue = queue.Queue()
        
        self.polling_thread = threading.Thread(target=self.poll_subscriptions, daemon=True)
        self.polling_thread.start()

    def poll_subscriptions(self):
        while True:
            try:
                identity, hb = self.hb_socket.recv_multipart(zmq.NOBLOCK)
                if hb == b"HB_ACK":
                    self.active_subscribers[identity] = time.time()
            except zmq.Again:
                pass
                
            events = dict(self.poller.poll(10))  # Reduced polling interval
            if self.pub_socket in events and events[self.pub_socket] == zmq.POLLIN:
                while True:
                    try:
                        message = self.pub_socket.recv_multipart(zmq.NOBLOCK)
                        if message[0][0:1] == b'\x01':
                            Master.no_of_subscribers += 1
                        elif message[0][0:1] == b'\x00':
                            Master.no_of_subscribers = max(0, Master.no_of_subscribers - 1)
                            self.visualization_queue.put("stop")
                            if Master.no_of_subscribers == 0:
                                self.oc.stop_visualization()
                    except zmq.Again:
                        break
            time.sleep(0.001)  # Reduced sleep

    def broadcast_tick(self):
        self.pub_socket.send_string("TICK", zmq.NOBLOCK)
        
        if Master.no_of_subscribers > 0:
            self.verify_acknowledgment()
        return True

    def verify_acknowledgment(self):
        self.waiting_for_ack = Master.no_of_subscribers
        self.conflict_counter = Master.no_of_subscribers
        timeout = 5.0  # Reduced timeout
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        occupancy_grids = {}

        while (self.waiting_for_ack > 0 or self.conflict_counter > 0) and (time.time() - start_time) < timeout:
            try:
                events = dict(poller.poll(50))  # Reduced poll timeout
                if self.sync_socket in events:
                    parts = self.sync_socket.recv_multipart(zmq.NOBLOCK)
                    msg_type = parts[0]

                    if msg_type == b"ACK":
                        self.sync_socket.send(b"ACK_RECEIVED", zmq.NOBLOCK)
                        self.waiting_for_ack -= 1

                    elif msg_type == b"CONFLICT_DETECTION":
                        self.conflict_counter -= 1
                        
                        if len(parts) > 1:
                            subscriber_id, grid = pickle.loads(parts[1])
                            occupancy_grids[subscriber_id] = grid
                            self.sync_socket.send(b"GRID_RECEIVED", zmq.NOBLOCK)
                        else:
                            self.sync_socket.send(b"ERROR_MISSING_DATA", zmq.NOBLOCK)
                            continue

                        if self.conflict_counter == 0:
                            # # Remove subscriber_data entries whose keys are not in occupancy_grids
                            # for key in list(self.subscribers_data.keys()):
                            #     if key not in occupancy_grids:
                            #         del self.subscribers_data[key]
                            #         for sub_data in self.subscribers_data.values():
                            #             sub_data['car_value'] -= 2
                            #             sub_data['car_reach_value'] -= 2
                            #             sub_data['path_value'] += 2
                            
                            # Assign unique values for each grid
                            for idx, (subscriber_id, grid) in enumerate(occupancy_grids.items()):
                                if subscriber_id not in self.subscribers_data:
                                    new_car_value = ((Master.no_of_subscribers-1) * 2) + 2
                                    new_car_reach_value = new_car_value + 1
                                    new_path_value = -new_car_value
                                    grid[grid == 2] = new_car_value
                                    grid[grid == 3] = new_car_reach_value
                                    grid[grid == -2] = new_path_value
                                    self.subscribers_data[subscriber_id] = {
                                        'car_value': new_car_value, 
                                        'car_reach_value': new_car_reach_value, 
                                        'path_value': new_path_value
                                    }
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
                            merged_grid = np.empty(grid_shape, dtype=object)
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
                                    if abs(merged_grid[idx]) >= 2 or abs(value) >= 2:
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
                                        #################################################################################################################
                                        if merged_grid[idx] == 3 or merged_grid[idx] == 5:
                                            if not isinstance(temp_grid[idx], list) or len(temp_grid[idx]) == 0:
                                                temp_grid[idx] = [merged_grid[idx]]
                                            elif merged_grid[idx] not in temp_grid[idx]:
                                                temp_grid[idx].append(merged_grid[idx])
                                        if value == 3 or value == 5:
                                            if not isinstance(temp_grid[idx], list) or len(temp_grid[idx]) == 0:
                                                temp_grid[idx] = [value]
                                            elif value not in temp_grid[idx]:
                                                temp_grid[idx].append(value)
                                        #print(f"temp_grid[{idx}] after appending: {temp_grid[idx]}")
                                        if isinstance(temp_grid[idx], list) and 3 in temp_grid[idx] and 5 in temp_grid[idx]:
                                            conflict = True
                                        # if merged_grid[idx] >= 3 and value >= 5:
                                        #     temp_grid[idx] = [merged_grid[idx], value]
                                        #     conflict = True
                                        # if merged_grid[idx] >= 5 and value >= 3:
                                        #     temp_grid[idx] = [merged_grid[idx], value]
                                        #     conflict = True
                                        # elif merged_grid[idx] >= 3:
                                        #     temp_grid[idx] = [merged_grid[idx]]
                                        # elif value >= 5:
                                        #     temp_grid[idx] = [value]
                                        
                                        #####################################################################################################################
                                        
                                        if abs(visualization_grid[idx]) >= abs(value):
                                            temp_grid_visualization[idx] = visualization_grid[idx]
                                        else:
                                            temp_grid_visualization[idx] = value
                                        
                                        if visualization_grid[idx] in (2, 4):
                                            temp_grid_visualization[idx] = visualization_grid[idx]
                                        if value in (2, 4):
                                            temp_grid_visualization[idx] = value
                                            
                                    if merged_grid[idx] != value:
                                        merged_grid[idx] = [merged_grid[idx], value]
                                        visualization_grid[idx] = max(visualization_grid[idx], value)
                                        
                                        if visualization_grid_view[idx] == -2 or value == -2:
                                            visualization_grid_view[idx] = -2
                                        elif visualization_grid_view[idx] == 6 or value == 6:
                                            visualization_grid_view[idx] = 6
                                        elif visualization_grid_view[idx] == -4 or value == -4:
                                            visualization_grid_view[idx] = -4
                                        elif visualization_grid_view[idx] == 4 or value == 4:
                                            visualization_grid_view[idx] = 4
                                        elif visualization_grid_view[idx] == 2 or value == 2:
                                            visualization_grid_view[idx] = 2
                                        elif visualization_grid_view[idx] == 5 or value == 5:
                                            visualization_grid_view[idx] = 5

                                        elif visualization_grid_view[idx] == 3 or value == 3:
                                            visualization_grid_view[idx] = 3
                                        
                                        elif visualization_grid_view[idx] == 1 or value == 1:
                                            visualization_grid_view[idx] = 1
                            
                            # Process conflicts
                            if conflict:
                                if not hasattr(self, 'overlap_obs'):
                                    self.overlap_obs = []
                                    if len(self.overlap_obs) == 0: 
                                        for idx, cell in np.ndenumerate(temp_grid):
                                            if isinstance(cell, list) and 3 in cell and 5 in cell:
                                                temp_grid_visualization[idx] = 1
                                                self.overlap_obs.append(idx)
                                else:
                                    for idx in self.overlap_obs:
                                        temp_grid_visualization[idx] = 1
                                        #self.overlap_obs.append(idx)
                                
                                            
                                if None not in (top_left, top_right, bottom_left, bottom_right):
                                    min_row = min(top_left[0], bottom_left[0])
                                    max_row = max(top_right[0], bottom_right[0])
                                    min_col = min(top_left[1], top_right[1])-5
                                    max_col = max(bottom_left[1], bottom_right[1])+5

                                    conflict_area = temp_grid_visualization[min_row:max_row+1, min_col:max_col+1]
                                    #overlapping_area = temp_grid[min_row:max_row+1, min_col:max_col+1]
                                
                                conflict_area, new_path_point = solve_conflict(conflict_area)
                                self.conflict_solved = {}
                                sub_id = None
                                
                                if new_path_point is not None:
                                    if np.any((conflict_area == 4)):
                                        for sub_id, sub_data in self.subscribers_data.items():
                                            if sub_data['car_value'] == 4:
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
                                        # output_dir = "output_occupancy_grids"
                                        # os.makedirs(output_dir, exist_ok=True)
                                        # filename = f"conflict_area_{subscriber_id}.npy"
                                        # filepath = os.path.join(output_dir, filename)
                                        # np.save(filepath, conflict_area)
                                    else:
                                        self.conflict_solved[subscriber_id] = 'No Conflict'
                            else:
                                self.conflict_solved = {subscriber_id: 'No Conflict' for subscriber_id in occupancy_grids.keys()}

                            
                                
                            #self.oc.update_visualization2(current_grid=visualization_grid_view)

                            if len(occupancy_grids) == 1:
                                #print("Only one occupancy grid received, no conflicts to resolve.")
                                self.oc.update_visualization2(current_grid=visualization_grid_view)
                            else:
                                self.oc.update_visualization2(current_grid=visualization_grid_view)
                                #self.oc.update_visualization2(current_grid=temp_grid_visualization)
                    
                    elif msg_type == b"SEND_SOLUTION":
                        # Keep blocking for pyobj as requested
                        self.sync_socket.send_pyobj(self.conflict_solved)

            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    pass
            except Exception:
                pass

        Master.no_of_subscribers = max(0, Master.no_of_subscribers - Master.pending_disconnects)
        Master.pending_disconnects = 0

    def close(self):
        try:
            for socket in [self.pub_socket, self.sync_socket, self.hb_socket]:
                socket.setsockopt(zmq.LINGER, 0)
                socket.close()
            self.context.term()
        except Exception:
            pass


class Subscriber:
    def __init__(self, sub_port=5555, sync_port=5556, hb_port=5557):
        self.context = zmq.Context()
        
        # Optimized Sub socket
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.SNDHWM, 10000)
        self.sub_socket.setsockopt(zmq.RCVHWM, 10000)
        self.sub_socket.setsockopt(zmq.LINGER, 0)
        self.sub_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.sub_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.sub_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.sub_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.sub_socket.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Optimized Sync socket
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.setsockopt(zmq.SNDHWM, 10000)
        self.sync_socket.setsockopt(zmq.RCVHWM, 10000)
        self.sync_socket.setsockopt(zmq.LINGER, 0)
        self.sync_socket.setsockopt(zmq.RCVTIMEO, 5000)  # Reduced timeout
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.sync_socket.connect(f"tcp://127.0.0.1:{sync_port}")
        
        # Optimized Heartbeat socket
        self.hb_socket = self.context.socket(zmq.DEALER)
        self.hb_socket.setsockopt(zmq.SNDHWM, 10000)
        self.hb_socket.setsockopt(zmq.RCVHWM, 10000)
        self.hb_socket.setsockopt(zmq.LINGER, 0)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 600)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)
        self.hb_socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.hb_socket.connect(f"tcp://localhost:{hb_port}")
        
        self.running = True
        self.uuid = str(uuid.uuid4())
        
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def send_heartbeat(self):
        while self.running:
            try:
                self.hb_socket.send(b"HB_ACK", zmq.NOBLOCK)
                time.sleep(1.0)  # Reduced interval
            except zmq.Again:
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)

    def receive_messages(self):
        if not self.running:
            return False

        try:
            if self.sub_socket.poll(10, zmq.POLLIN):  # Reduced timeout
                message = self.sub_socket.recv_string(zmq.NOBLOCK)
                return message == "TICK"
        except zmq.Again:
            pass
        except Exception:
            self.running = False
        return False

    def acknowledge_message(self):
        try:
            self.sync_socket.send(b"ACK", zmq.NOBLOCK)
            if self.sync_socket.poll(3000):  # Reduced timeout
                reply = self.sync_socket.recv(zmq.NOBLOCK)
                return True
        except Exception:
            self.reset_sync_socket()
            return False
        return False

    def send_conflict(self, occupancy_grid):
        max_retries = 2  # Reduced retries
        backoff = 0.05  # Reduced backoff

        for attempt in range(max_retries):
            try:
                self.sync_socket.send_multipart([
                    b"CONFLICT_DETECTION",
                    pickle.dumps((self.uuid, occupancy_grid))
                ], zmq.NOBLOCK)

                if self.sync_socket.poll(3000):  # Reduced timeout
                    reply = self.sync_socket.recv(zmq.NOBLOCK)
                    if reply == b"GRID_RECEIVED":
                        return True
                    elif reply == b"ERROR_MISSING_DATA":
                        time.sleep(backoff)
                        backoff *= 2
                        continue
                    
            except Exception:
                time.sleep(backoff)
                backoff *= 2
                self.reset_sync_socket()

        return False
    
    def receive_solution(self):
        try:
            self.sync_socket.send(b"SEND_SOLUTION", zmq.NOBLOCK)
            if self.sync_socket.poll(3000):  # Reduced timeout
                # Keep blocking for pyobj as requested
                reply = self.sync_socket.recv_pyobj()
                if reply is None:
                    return None
                if isinstance(reply[self.uuid], str) and reply[self.uuid] == 'No Conflict':
                    return None
                elif isinstance(reply[self.uuid], dict):
                    return reply[self.uuid]
        except Exception:
            self.reset_sync_socket()
        return None   

    def reset_sync_socket(self):
        try:
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.disconnect(f"tcp://127.0.0.1:5556")
            self.sync_socket.close()
            time.sleep(0.1)  # Reduced sleep
            
            if self.context.closed:
                self.context = zmq.Context()
                
            self.sync_socket = self.context.socket(zmq.REQ)
            self.sync_socket.setsockopt(zmq.SNDHWM, 10000)
            self.sync_socket.setsockopt(zmq.RCVHWM, 10000)
            self.sync_socket.setsockopt(zmq.LINGER, 0)
            self.sync_socket.setsockopt(zmq.RCVTIMEO, 5000)
            self.sync_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
            self.sync_socket.connect(f"tcp://127.0.0.1:5556")
        except Exception:
            self.context = zmq.Context()
            self.sync_socket = self.context.socket(zmq.REQ)

    def close(self):
        self.running = False
        try:
            for socket in [self.sub_socket, self.sync_socket, self.hb_socket]:
                socket.setsockopt(zmq.LINGER, 0)
                socket.close()
            self.context.term()
        except Exception:
            pass


# Optimized main execution
if __name__ == "__main__":
    master = Master()

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)  # Reduced timeout
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    try:
        while True:
            world.tick()
            master.broadcast_tick()
            
            try:
                while not master.visualization_queue.empty():
                    cmd = master.visualization_queue.get_nowait()
                    if cmd == "stop":
                        master.oc.stop_visualization()
            except Exception:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        master.close()
