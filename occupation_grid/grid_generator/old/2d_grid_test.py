import carla
import numpy as np
import cv2

def segment_parking_lines(large_bb):
    # Get the main bounding box dimensions
    start_x = large_bb.location.x - large_bb.extent.x
    end_x = large_bb.location.x + large_bb.extent.x
    y_pos = large_bb.location.y
    
    # Typical parking line width
    line_width = 0.15  # meters
    
    # Distance between parking spots (adjust based on your map)
    spot_distance = 2.5  # meters
    
    individual_lines = []
    current_x = start_x
    
    while current_x < end_x:
        # Create bounding box for each line
        line_location = carla.Location(x=current_x, y=y_pos, z=large_bb.location.z)
        line_bb = carla.BoundingBox(line_location, carla.Vector3D(line_width/2, 0.1, 0.01))
        individual_lines.append(line_bb)
        
        # Move to next line
        current_x += spot_distance
        
    return individual_lines


def create_2d_grid(world, grid_size=500, cell_size=1):
    # Create an empty grid
    grid = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)
    
    # Calculate the grid center
    center = grid_size // 2
    
    # Get bounding boxes for static objects (e.g., walls)
    walls = world.get_level_bbs(carla.CityObjectLabel.Other)
    parking_spots = world.get_level_bbs(carla.CityObjectLabel.RoadLines)

    print(len(walls))
    print(len(parking_spots))


    # level_bbs =  walls #+ parking_spots
    level_bbs = parking_spots
    print(len(level_bbs))
    # Draw bounding boxes on the grid
    for bb in level_bbs:
        # Get the center location and extent of the bounding box
        bb_center = bb.location
        bb_extent = bb.extent
        
        # Get the rotation of the bounding box
        rotation = bb.rotation
        
        # Calculate the corners of the bounding box in world coordinates
        corners = get_bounding_box_corners(bb_center, bb_extent, rotation)
        
        # Convert corners to grid coordinates
        grid_corners = []
        for corner in corners:
            x = int(center + corner.x / cell_size)
            y = int(center + corner.y / cell_size)
            grid_corners.append((x, y))
        
        # Draw the rotated bounding box as a polygon
        cv2.polylines(grid, [np.array(grid_corners, dtype=np.int32)], isClosed=True, color=(255, 0, 0), thickness=1)
    
    return grid

def get_bounding_box_corners(center, extent, rotation):
    """
    Calculate the 8 corners of a rotated bounding box in world coordinates.
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

client = carla.Client('localhost', 2000)
world = client.get_world()
grid = create_2d_grid(world)
cv2.imwrite('grid.png', grid)

# def main():
#     try:
#         # Connect to CARLA server
#         client = carla.Client('localhost', 2000)
#         # client.set_timeout(2.0)
        
#         world = client.get_world()
        
#         while True:
#             # Create and display a 2D grid with rotated bounding boxes
#             grid = create_2d_grid(world)
#             cv2.imshow('CARLA Rotated Bounding Boxes', grid)
            
#             # Break loop on 'q' press
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
                
#     finally:
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
