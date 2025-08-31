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
import queue
import argparse
import csv
import cv2


class Master:
    no_of_subscribers = 0
    pending_disconnects = 0

    def __init__(self, pub_port=5555, sync_port=5556, hb_port=5557, visualize=False, collect_data=False):
        self.visualize = visualize
        self.collect_data = collect_data
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
        if self.visualize:
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
        self.overlap_obs = None
        self.decision_to_make = True
        self.training_data_no = 0
        self.time_step_data = 0
        

        if self.visualize:
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
                            if self.visualize:
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
    
    def _priority_update(self, vis, new, diff_mask):
        # Define priority mapping
        pm = {-2: 8, 6: 7, -4: 6, 4: 5, 2: 4, 5: 3, 3: 2, 1: 1, 0: 0}
        vec = np.vectorize(pm.get)
        vp = vec(vis)
        np_ = vec(new)
        mask = diff_mask & (np_ > vp)
        return mask, new[mask]
    
    def decision_maker_helper(self, grid, direction):
        # 1. find leftmost column index of any –2
        path_locs = np.where(grid == -2)
        if path_locs[1].size == 0:
            raise ValueError("No -2 found in grid")
        sidemost = path_locs[1].min()

        # 2. build a mask for any 3 in columns strictly left of that
        cols = np.arange(grid.shape[1])[None, :]    # shape (1, W)
        if direction == 'L':
            mask = (grid == 3) & (cols < sidemost)
        else:  # direction == 'R'
            mask = (grid == 3) & (cols > sidemost)

        # 3. zero out those positions
        grid[mask] = 0
        return grid
    
    def check_if_car_fit(self, grid):
        """
        Check if the pattern of 2s can fit anywhere within the 3s regions.
        Returns True if fit is possible, False otherwise.
        """
        # Get mask of 2s and 3s
        twos_mask = (grid == 2)
        threes_mask = (grid == 3)
        
        if not np.any(twos_mask):
            return True  # No 2s to fit
        if not np.any(threes_mask):
            return False  # No 3s available for fitting
        
        # Get bounding box of the 2s pattern
        twos_positions = np.argwhere(twos_mask)
        min_row, min_col = twos_positions.min(axis=0)
        max_row, max_col = twos_positions.max(axis=0)
        
        # Extract the 2s pattern (relative to its bounding box)
        pattern_height = max_row - min_row + 1
        pattern_width = max_col - min_col + 1
        twos_pattern = twos_mask[min_row:max_row+1, min_col:max_col+1]
        
        # Try to fit the pattern at every possible position in the 3s
        grid_height, grid_width = grid.shape
        
        for start_row in range(grid_height - pattern_height + 1):
            for start_col in range(grid_width - pattern_width + 1):
                # Extract the region from the 3s mask
                end_row = start_row + pattern_height
                end_col = start_col + pattern_width
                region_threes = threes_mask[start_row:end_row, start_col:end_col]
                
                # Check if the 2s pattern fits completely within 3s
                if np.all(region_threes[twos_pattern]):
                    #print(f"Fit found at position ({start_row}, {start_col})")
                    return True  # Found a fit!
        
        #print("No fit found")
        return False  # No fit found

    def verify_acknowledgment(self):
        self.waiting_for_ack = Master.no_of_subscribers
        self.conflict_counter = Master.no_of_subscribers
        timeout = 5.0  # Reduced timeout
        start_time = time.time()
        poller = zmq.Poller()
        poller.register(self.sync_socket, zmq.POLLIN)
        occupancy_grids = {}
        decision_grids = {}

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
                            occupancy_grids[subscriber_id] = grid[0]
                            decision_grids[subscriber_id] = grid[1]
                            self.sync_socket.send(b"GRID_RECEIVED", zmq.NOBLOCK)
                        else:
                            self.sync_socket.send(b"ERROR_MISSING_DATA", zmq.NOBLOCK)
                            continue

                        if self.conflict_counter == 0:
                            if self.collect_data:
                                solver_time_start = time.time()
                            # # Remove subscriber_data entries whose keys are not in occupancy_grids
                            # for key in list(self.subscribers_data.keys()):
                            #     if key not in occupancy_grids:
                            #         del self.subscribers_data[key]
                            #         for sub_data in self.subscribers_data.values():
                            #             sub_data['car_value'] -= 2
                            #             sub_data['car_reach_value'] -= 2
                            #             sub_data['path_value'] += 2

                            # Assign unique values vectorized
                            for subscriber_id, grid in occupancy_grids.items():
                                if subscriber_id not in self.subscribers_data:
                                    new_car = ((Master.no_of_subscribers - 1) * 2) + 2
                                    new_reach = new_car + 1
                                    new_path = -new_car
                                    grid[grid == 2] = new_car
                                    grid[grid == 3] = new_reach
                                    grid[grid == -2] = new_path
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == 2] = new_car
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == 3] = new_reach
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == -2] = new_path
                                    self.subscribers_data[subscriber_id] = {
                                        'car_value': new_car,
                                        'car_reach_value': new_reach,
                                        'path_value': new_path
                                    }
                                else:
                                    sd = self.subscribers_data[subscriber_id]
                                    grid[grid == 2] = sd['car_value']
                                    grid[grid == 3] = sd['car_reach_value']
                                    grid[grid == -2] = sd['path_value']
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == 2] = sd['car_value']
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == 3] = sd['car_reach_value']
                                    # decision_grids[subscriber_id][decision_grids[subscriber_id] == -2] = sd['path_value']



                            # np.save("fast_grid/grid.npy", grid)
                            grids = list(occupancy_grids.values())
                            decisiopn_grids = list(decision_grids.values())
                            shape = grids[0].shape

                            merged = grids[0].copy()
                            merged_decision = decisiopn_grids[0].copy()
                            vis_grid = grids[0].copy()
                            conflict_mask = np.zeros(shape, dtype=bool)

                            # Merge all grids
                            significant_mask = np.zeros(shape, dtype=bool)
                            for grid, decision_grid in zip(grids[1:], decisiopn_grids[1:]):
                                # Update merged + detect conflict
                                reach3 = (merged == 3) | (merged == 5)
                                reach5 = (grid == 3) | (grid == 5)
                                conflict_mask |= reach3 & reach5
                                significant_mask |= (np.abs(merged_decision) >= 2) | (np.abs(decision_grid) >= 2)

                                # Update merged where different
                                diff = merged != grid
                                merged[diff] = grid[diff]

                                # Visualization: pick max abs value
                                higher = np.abs(grid) > np.abs(vis_grid)
                                vis_grid[higher] = grid[higher]

                                # Prioritize car markers (2,4)
                                car_spots = np.isin(grid, [2, 4])
                                vis_grid[car_spots] = grid[car_spots]

                                # Then apply full priority map
                                pmask, pvals = self._priority_update(vis_grid, grid, diff)
                                vis_grid[pmask] = pvals


                            

                            # if not os.path.exists("fast_grid"):
                            #     os.makedirs("fast_grid")

                            # np.save("fast_grid/vis_grid.npy", vis_grid)
                            visualization_grid_view = vis_grid.copy()

                            # conflict_area = None
                            
                            # Process conflicts
                            if conflict_mask.any():

                                #print("Conflicts detected, solving...")
                                
                                if self.overlap_obs is None:
                                    self.overlap_obs = conflict_mask
                                    # coords = np.where(conflict_mask)
                                    # self.overlap_obs = list(zip(coords[0], coords[1]))

                                vis_grid[self.overlap_obs] = 1  # mark conflicts               
                                
                                            
                                rows, cols = np.where(significant_mask)
                                min_r, max_r = rows.min(), rows.max()
                                min_c = max(cols.min() , 0)#-5
                                max_c = min(cols.max() , vis_grid.shape[1] - 1)# + 5

                                conflict_area = vis_grid[min_r:max_r+1, min_c:max_c+1]

                                # Save photo of the conflict area before solving
                                if self.collect_data:
                                    abs_current_grid = np.abs(conflict_area)
                                    colored_grid = self.oc.color_map[abs_current_grid]
                                    if not os.path.exists("photos/conflict_area_before"):
                                        os.makedirs("photos/conflict_area_before")
                                    cv2.imwrite(f"photos/conflict_area_before/conflict_area_{self.time_step_data}.png", colored_grid)
                                    # self.time_step_data += 1

                                if self.decision_to_make:
                                    
                                    # if not os.path.exists("decision_grids"):
                                    #     os.makedirs("decision_grids")

                                    no_of_cars = Master.no_of_subscribers - 1

                                    for subscriber_id, grid in decision_grids.items():
                                        # np.save(f"decision_grids/decision_grids_{subscriber_id}.npy", grid)
                                        grid = grid[min_r:max_r+1, min_c:max_c+1]
                                        indices = np.argwhere(grid == -2)
                                        #print(len(indices))
                                        if indices.size > 0:
                                            first_idx = indices[0]
                                            last_idx = indices[-1]
                                            neighbors = []
                                            for idx in [first_idx, last_idx]:
                                                r, c = idx
                                                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                                                    nr, nc = r+dr, c+dc
                                                    if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
                                                        neighbors.append((nr, nc))
                                                    else:
                                                        neighbors.append(None)
                                            # Check which neighbor has value 2
                                            #print(neighbors)
                                            direction = None
                                            for idx, neighbor in enumerate(neighbors):
                                                if neighbor is not None:
                                                    nr, nc = neighbor
                                                    #print(nr, nc, grid[nr, nc])
                                                    if grid[nr, nc] == 2:
                                                        if (idx+1)%4 == 1:  # top neighbor
                                                            #print("top")
                                                            direction = 'R'
                                                        elif (idx+1)%4 == 2:  # bottom neighbor
                                                            #print("bottom")
                                                            direction = 'L'
                                                        elif (idx+1)%4 == 3:  # left neighbor
                                                            #print("left")
                                                            direction = 'U'
                                                        elif (idx+1)%4 == 4:  # right neighbor
                                                            #print("right")
                                                            direction = 'D'
                                                        else:
                                                            print("No direction found")
                                                        break
                                            # neighbors now contains the up/down/left/right neighbors of first and last indices
                                        decision_grid = self.decision_maker_helper(grid, direction=direction)
                                        check_fit = self.check_if_car_fit(decision_grid)
                                        if check_fit:
                                            car = ((Master.no_of_subscribers - 1) * 2) + 2
                                            self.subscribers_data[subscriber_id] = {
                                            'car_value': car,
                                            'car_reach_value': car + 1,
                                            'path_value': -car
                                            }
                                        else:
                                            car = ((no_of_cars - 1) * 2) + 2
                                            self.subscribers_data[subscriber_id] = {
                                            'car_value': car,
                                            'car_reach_value': car + 1,
                                            'path_value': -car
                                            }
                                            no_of_cars -= 1                                            

                                        # np.save(f"decision_grids/decision_grids_{subscriber_id}.npy", grid)

                                        
                                    print("Decisions saved for all subscribers.")
                                    self.decision_to_make = False

                                # self.oc.update_visualization2(current_grid=conflict_area)
                                # self.conflict_solved = {subscriber_id: 'No Conflict' for subscriber_id in occupancy_grids.keys()}

                                #conflict_area_temp = conflict_area.copy()
                                
                                conflict_area, new_path_point, waypoint_og = solve_conflict(conflict_area)

                                # Save photo of the conflict area after solving
                                if self.collect_data:
                                    abs_current_grid = np.abs(conflict_area)
                                    colored_grid = self.oc.color_map[abs_current_grid]
                                    if not os.path.exists("photos/conflict_area_after"):
                                        os.makedirs("photos/conflict_area_after")
                                    cv2.imwrite(f"photos/conflict_area_after/conflict_area_{self.time_step_data}.png", colored_grid)
                                    self.time_step_data += 1

                                # if waypoint_og is not None:
                                #     if not os.path.exists("training_data"):
                                #         os.makedirs("training_data")
                                #     np.save(f"training_data/visualization_{self.training_data_no}", visualization_grid_view)
                                #     self.training_data_no += 1
                                #     with open("training_data/waypoints.txt", "a") as f:
                                #         f.write(f"{new_path_point[0] + min_r},{new_path_point[1] + min_c}\n")
                                #         #f.write(f"{waypoint_og[0] + min_r},{waypoint_og[1] + min_c}\n")
                                

                                visualization_grid_view[waypoint_og[0] + min_r, waypoint_og[1] + min_c] = 7
           

                                # self.conflict_solved = {}
                                # sub_id = None
                                # print('New path point:', new_path_point)
                                
                                # Identify subscriber whose car_value == 4 (if any)
                                sub_with_4 = next(
                                    (sid for sid, sd in self.subscribers_data.items()
                                    if sd['car_value'] == 4),
                                    None
                                )
                                # if new_path_point is not None:
                                #     if np.any((conflict_area == 4)):
                                #         for sub_id, sub_data in self.subscribers_data.items():
                                #             if sub_data['car_value'] == 4:
                                #                 break
                                # print('sub_with_4:', sub_id)
                                
                                for sid in occupancy_grids:
                                    if sid == sub_with_4:
                                        conflict_area[conflict_area == 4] = 0
                                        self.conflict_solved[sid] = {
                                            'conflict_area': conflict_area,
                                            'new_path_point': new_path_point,
                                            'conflict_area_bounds': {
                                                'min_row': min_r, 'max_row': max_r,
                                                'min_col': min_c, 'max_col': max_c
                                            }
                                        }
                                    else:
                                        self.conflict_solved[sid] = 'No Conflict'

                            else:
                                self.conflict_solved = {subscriber_id: 'No Conflict' for subscriber_id in occupancy_grids.keys()}
        
                            if self.collect_data:
                                solver_time_end = time.time()

                                if solver_time_start is not None and solver_time_end is not None:
                                    csv_path = "csv_time_data/solver.csv"
                                    solver_time = solver_time_end - solver_time_start
                                    with open(csv_path, "a", newline="") as csvfile:
                                        writer = csv.writer(csvfile)
                                        writer.writerow([solver_time_start, solver_time_end, solver_time])
                                    solver_time_start = None
                                    solver_time_end = None

                            
                                
                            #self.oc.update_visualization2(current_grid=vis_grid)
                            if self.visualize:
                                #self.oc.update_visualization2(current_grid=vis_grid)
                                # self.oc.update_visualization2(current_grid=visualization_grid_view)

                                # if conflict_area is None:
                                #     visual = vis_grid
                                # else:
                                #     visual = conflict_area


                                # Add a black line (value 1) between the grids
                                separator = np.ones((visualization_grid_view.shape[0], 2), dtype=visualization_grid_view.dtype)
                                # Concatenate visualization_grid_view, separator, and temp_grid_visualization horizontally
                                combined_grid = np.concatenate(
                                    (visualization_grid_view, separator, vis_grid), axis=1
                                )
                                # Visualize the combined grid
                                self.oc.update_visualization2(current_grid=combined_grid, zoom=False)

                            #     if len(occupancy_grids) == 1:
                            #         #print("Only one occupancy grid received, no conflicts to resolve.")
                            #         self.oc.update_visualization2(current_grid=visualization_grid_view)
                            #     else:
                            #         #self.oc.update_visualization2(current_grid=visualization_grid_view)
                            #         #self.oc.update_visualization2(current_grid=temp_grid_visualization)

                            #         # Add a black line (value 1) between the grids
                            #         separator = np.ones((visualization_grid_view.shape[0], 2), dtype=visualization_grid_view.dtype)
                            #         # Concatenate visualization_grid_view, separator, and temp_grid_visualization horizontally
                            #         combined_grid = np.concatenate(
                            #             (visualization_grid_view, separator, temp_grid_visualization), axis=1
                            #         )
                            #         # Visualize the combined grid
                            #         self.oc.update_visualization2(current_grid=combined_grid, zoom=False)

                    
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', help='Enable visualization')
    parser.add_argument('--collect_data', action='store_true', help='Enable data collection')
    args = parser.parse_args()
    master = Master(visualize=args.visualize, collect_data=args.collect_data)

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
            if args.visualize:
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
