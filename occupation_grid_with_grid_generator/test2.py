import numpy as np
import carla
import cv2
import random
import time
import subprocess
import threading
import xml.etree.ElementTree as ET
import math


# Dummy bounding box class
class SimpleBoundingBox:
    def __init__(self, location, extent, rotation):
        self.location = location  # carla.Location assumed local
        self.extent = extent      # carla.Vector3D half sizes
        self.rotation = rotation  # carla.Rotation local rotation (likely zero)

def get_parking_space_transform(carla_map, s, t, hdg, road_id=12):
    waypoints = carla_map.generate_waypoints(distance=0.25)

    # If road_id is known, filter waypoints by it for more robust selection
    if road_id is not None:
        waypoints = [wp for wp in waypoints if wp.road_id == road_id]
        if not waypoints:
            waypoints = carla_map.generate_waypoints(distance=1)  # fallback

    # Heuristic: order waypoints by progression along their road (in OpenDrive this is s), so for each sequence of road_id, sort by projection along heading.
    # In practice, we select the waypoint whose projection along its tangent vector (lane direction) is closest to s.

    # Try to approximately "align" the s coordinate to the waypoint sequence for each road
    closest_wp = min(
        waypoints, 
        key=lambda wp: abs((wp.road_id if road_id else 0) - (road_id if road_id is not None else 0)) + abs(wp.transform.location.x - s)
    )

    yaw = math.radians(closest_wp.transform.rotation.yaw)
    lateral_offset = carla.Vector3D(-math.sin(yaw), math.cos(yaw), 0)
    # Apply t as lateral offset from the centerline
    global_location = closest_wp.transform.location + lateral_offset * t

    # hdg in OpenDrive is relative to the road's centerline direction, so add it
    parking_yaw = closest_wp.transform.rotation.yaw + math.degrees(hdg)
    parking_rotation = carla.Rotation(pitch=0, yaw=parking_yaw, roll=0)

    return carla.Transform(global_location, parking_rotation)


def create_2d_obstacle_grid(world, grid_size=500, cell_size=0.5):
    # Create an empty grid
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    
    # Calculate the grid center
    center = grid_size // 2
    
    # Get bounding boxes for static objects (e.g., walls)
    walls = world.get_level_bbs(carla.CityObjectLabel.Other)
    
    # Mark obstacles on the grid
    for bb in walls:
        mark_bounding_box(grid, bb, center, cell_size, value=1)

    # Assuming you have a CARLA client and world object already:
    carla_map = world.get_map()

    # Get the OpenDrive XML as a string
    open_drive_xml = carla_map.to_opendrive()

    root = ET.fromstring(open_drive_xml)

    # #root = tree.getroot()

    # for obj in root.findall(".//object[@name='Stencil_Parking3']"):
    #     s = float(obj.attrib['s'])
    #     t = float(obj.attrib['t'])
    #     hdg = float(obj.attrib['hdg'])
    #     width = float(obj.attrib['width'])
    #     length = float(obj.attrib['length'])

    #     transform = get_parking_space_transform(carla_map, s, t, hdg)
    #     # Create bounding box located at origin, since transform will handle positioning
    #     local_location = carla.Location(x=0, y=0, z=0)  # bounding box center at origin
    #     extent = carla.Vector3D(length / 2, width / 2, 0.1)  # half sizes

    #     bb = SimpleBoundingBox(local_location, extent, carla.Rotation())  # no local rotation

    #     mark_bounding_box(grid, bb, center, cell_size, value=1, transform=transform)


    for obj in root.findall(".//object"):
        name = obj.attrib.get('name', '')
        obj_type = obj.attrib.get('type', '')
        s = float(obj.attrib.get('s', 0))
        t = float(obj.attrib.get('t', 0))
        hdg = float(obj.attrib.get('hdg', 0))
        width = float(obj.attrib.get('width', 0))
        length = float(obj.attrib.get('length', 0))
    

        # You may choose to filter only parking lines or spaces, e.g.:
        if 'parking' in name.lower() or obj_type == 'parkingSpace':

            # Your transform and bounding box construction logic here
            transform = get_parking_space_transform(carla_map, s, t, hdg)
            local_location = carla.Location(x=0, y=0, z=0)
            extent = carla.Vector3D(length / 2, width / 2, 0.1)
            bb = SimpleBoundingBox(local_location, extent, carla.Rotation())

            mark_bounding_box(grid, bb, center, cell_size, value=1, transform=transform)



    
    return grid

def mark_ego_vehicle(grid, ego_vehicle, center, cell_size):
    # Create a copy of the grid
    new_grid = grid.copy()
    
    # Mark ego vehicle's bounding box
    ego_bb = ego_vehicle.bounding_box
    ego_transform = ego_vehicle.get_transform()
    mark_bounding_box(new_grid, ego_bb, center, cell_size, value=2, transform=ego_transform)
    
    return new_grid

def mark_bounding_box(grid, bb, center, cell_size, value, transform=None):
    if transform:
        bb_center = transform.transform(bb.location)
        rotation = transform.rotation
    else:
        bb_center = bb.location
        rotation = bb.rotation
    
    bb_extent = bb.extent
    
    # Calculate the corners of the rotated bounding box
    corners = get_bounding_box_corners(bb_center, bb_extent, rotation)
    
    # Convert corners to grid coordinates
    grid_corners = []
    for corner in corners:
        x = int(center + corner.x / cell_size)
        y = int(center + corner.y / cell_size)
        grid_corners.append((x, y))
    
    # Create a mask for the rotated bounding box
    mask = np.zeros(grid.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(grid_corners, dtype=np.int32)], value)
    
    # Apply the mask to the grid
    grid[mask == value] = value

def get_bounding_box_corners(center, extent, rotation):
    """
    Calculate the 4 corners of a rotated bounding box in world coordinates.
    """
    # Convert rotation to radians
    yaw_rad = np.radians(rotation.yaw)
    
    # Define the relative positions of corners in local space (2D projection)
    local_corners = [
        carla.Location(x=-extent.x, y=-extent.y),
        carla.Location(x= extent.x, y=-extent.y),
        carla.Location(x= extent.x, y= extent.y),
        carla.Location(x=-extent.x, y= extent.y),
    ]
    
    # Rotate and translate corners to world space
    world_corners = []
    for corner in local_corners:
        # Apply rotation (yaw only for 2D visualization)
        rotated_x = corner.x * np.cos(yaw_rad) - corner.y * np.sin(yaw_rad)
        rotated_y = corner.x * np.sin(yaw_rad) + corner.y * np.cos(yaw_rad)
        
        # Translate to world position (center of the bounding box)
        world_corner = carla.Location(
            x=center.x + rotated_x,
            y=center.y + rotated_y,
            z=center.z  # Z is ignored for 2D visualization
        )
        world_corners.append(world_corner)
    
    return world_corners

def visualize_grid_animated(grid, ego_vehicle, zoom_factor=2, context_size=200):
    color_map = np.array([[255, 255, 255], [0, 0, 0], [255, 0, 0]], dtype=np.uint8)
    
    cv2.namedWindow('Animated Obstacle Grid', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Animated Obstacle Grid', 800, 800)
    
    while True:
        # Mark ego vehicle on a copy of the grid
        current_grid = mark_ego_vehicle(grid, ego_vehicle, grid.shape[0] // 2, 1)
        
        # Apply the color map
        colored_grid = color_map[current_grid]
        
        # Find ego vehicle position
        ego_positions = np.where(current_grid == 2)
        
        if len(ego_positions[0]) > 0:
            center_y, center_x = ego_positions[0].mean(), ego_positions[1].mean()
            
            # Calculate the visible area
            start_y = max(0, int(center_y - context_size // 2))
            end_y = min(grid.shape[0], int(center_y + context_size // 2))
            start_x = max(0, int(center_x - context_size // 2))
            end_x = min(grid.shape[1], int(center_x + context_size // 2))
            
            # Adjust if too close to the edges
            if end_y >= grid.shape[0] - 10:
                shift = end_y - (grid.shape[0] - 10)
                start_y = max(0, start_y - shift)
                end_y = grid.shape[0]
            if start_y <= 10:
                shift = 10 - start_y
                end_y = min(grid.shape[0], end_y + shift)
                start_y = 0
            if end_x >= grid.shape[1] - 10:
                shift = end_x - (grid.shape[1] - 10)
                start_x = max(0, start_x - shift)
                end_x = grid.shape[1]
            if start_x <= 10:
                shift = 10 - start_x
                end_x = min(grid.shape[1], end_x + shift)
                start_x = 0
            
            context_grid = colored_grid[start_y:end_y, start_x:end_x]
            
            # Zoom in
            zoomed_grid = cv2.resize(context_grid, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_NEAREST)
            
            cv2.imshow('Animated Obstacle Grid', zoomed_grid)
        else:
            cv2.imshow('Animated Obstacle Grid', colored_grid)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(0.1)  # Adjust this to control animation speed

    cv2.destroyAllWindows()

# Main execution
client = carla.Client('localhost', 2000)
world = client.get_world()
# vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
# spawn_points = world.get_map().get_spawn_points()
# ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

# Get all actors in the world
all_actors = world.get_actors()

# Filter for vehicles
vehicles = all_actors.filter('vehicle.*')
print(vehicles)
if len(vehicles)> 0:
    ego_vehicle=vehicles[0]

# # Find the vehicle with the matching role_name
# target_vehicle_name = "hero"
# ego_vehicle = None

# for vehicle in vehicles:
#     if vehicle.attributes.get('role_name') == target_vehicle_name:
#         ego_vehicle = vehicle
#         break

# if ego_vehicle is not None:
#     print(f"Found vehicle: {ego_vehicle.id}")
# else:
#     print("Vehicle not found")

# Create the static obstacle grid
static_obstacle_grid = create_2d_obstacle_grid(world)


# Save the static obstacle grid to a file
np.save('final_grid.npy', static_obstacle_grid)


# Start visualization in a separate thread

#vis_thread = threading.Thread(target=visualize_grid_animated, args=(static_obstacle_grid, ego_vehicle, 2, 200))
#vis_thread.start()

# ego_vehicle.set_autopilot(True)
# # Run the manual_control.py script
# control_thread = threading.Thread(target=subprocess.run, args=(['python', 'manual_control.py', '--rolename="hero"'],))
# control_thread.start()

# # Get all actors in the world
# all_actors = world.get_actors()

# # Filter for vehicles
# vehicles = all_actors.filter('vehicle.*')

# # Find the vehicle with the matching name
# target_vehicle_name = "hero"
# ego_vehicle = None

# for vehicle in vehicles:
#     if vehicle.attributes.get('role_name') == target_vehicle_name:
#         ego_vehicle = vehicle
#         break
# if ego_vehicle:
#     print(f"Found vehicle: {ego_vehicle.id}")
# else:
#     print("Vehicle not found")







# # Move the ego vehicle
# while vis_thread.is_alive():
#     # Simple random movement
#     control = carla.VehicleControl()
#     control.throttle = random.uniform(0.3, 0.7)
#     control.steer = random.uniform(-0.3, 0.3)
#     ego_vehicle.apply_control(control)
#     time.sleep(0.1)

# ego_vehicle.destroy()
