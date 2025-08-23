import numpy as np
import carla
import matplotlib.pyplot as plt
import cv2
import random
import time


def get_bounding_box_center(bounding_box):
    """
    Get the center of the bounding box in world coordinates.
    """
    return bounding_box.location

def create_2d_obstacle_grid(world, ego_bounding_box, grid_size=500, cell_size=1):
    # Create an empty grid
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    
    # Calculate the grid center
    center = grid_size // 2
    
    # Get bounding boxes for static objects (e.g., walls)
    walls = world.get_level_bbs(carla.CityObjectLabel.Other)
    # roadlines = world.get_level_bbs(carla.CityObjectLabel.RoadLines)
    
    # Combine all bounding boxes
    all_bbs = walls 
    
    # Mark obstacles on the grid
    for bb in all_bbs:
        # Get the center location, extent, and rotation of the bounding box
        bb_center = bb.location
        bb_extent = bb.extent
        rotation = bb.rotation
        
        # Calculate the corners of the rotated bounding box
        corners = get_bounding_box_corners(bb_center, bb_extent, rotation)
        
        # Convert corners to grid coordinates
        grid_corners = []
        for corner in corners:
            x = int(center + corner.x / cell_size)
            y = int(center + corner.y / cell_size)
            grid_corners.append((x, y))
        
        # Create a mask for the rotated bounding box
        mask = np.zeros((grid_size, grid_size), dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(grid_corners, dtype=np.int32)], 1)
        
        # Apply the mask to the grid
        grid = np.logical_or(grid, mask).astype(np.uint8)

    # Add ego vehicle's bounding box center to the grid
    ego_center = get_bounding_box_center(ego_bounding_box)
    ego_grid_x = int(center + ego_center.x / cell_size)
    ego_grid_y = int(center + ego_center.y / cell_size)
    
    # Ensure the coordinates are within the grid
    ego_grid_x = max(0, min(ego_grid_x, grid_size - 1))
    ego_grid_y = max(0, min(ego_grid_y, grid_size - 1))
    
    # Mark the ego vehicle's center on the grid
    grid[ego_grid_y, ego_grid_x] = 1
    
    return grid


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

def get_obstacle_lists(grid):
    ox, oy = [], []
    rows, cols = grid.shape
    
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] == 1:
                ox.append(x)
                oy.append(y)
    
    return ox, oy


# def get_obstacle_lists(grid):
#     ox, oy = [], []
#     rows, cols = grid.shape
    
#     for y in range(rows):
#         for x in range(cols):
#             if grid[y, x] == 1:
#                 ox.append(x)
#                 oy.append(y)
    
#     return ox, oy

def plot_map(ox, oy):
    
    plt.cla()
    plt.plot(ox, oy, "sk")
    plt.axis("equal")
    # plt.plot(x, y, linewidth=1.5, color='r')
    plt.show()
    print("Done!")

client = carla.Client('localhost', 2000)
world = client.get_world()
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
spawn_points = world.get_map().get_spawn_points()
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
ego_bounding_box = ego_vehicle.bounding_box
time.sleep(5)
grid = create_2d_obstacle_grid(world, ego_bounding_box)
ox, oy = get_obstacle_lists(grid)


plot_map(ox, oy)
