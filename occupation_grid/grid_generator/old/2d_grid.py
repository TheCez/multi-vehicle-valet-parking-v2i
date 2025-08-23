import carla
import numpy as np
import cv2

def create_2d_grid(world, grid_size=500, cell_size=1):
    # Create an empty grid
    grid = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)
    
    # Calculate the grid center
    center = grid_size // 2
    
    # Get bounding boxes for static objects (e.g., walls)
    level_bbs = world.get_level_bbs(carla.CityObjectLabel.Other)

    # Draw bounding boxes on the grid
    for bb in level_bbs:
        # Convert bounding box corners to grid coordinates
        bb_min = bb.location - bb.extent  # Min corner of the bounding box
        bb_max = bb.location + bb.extent  # Max corner of the bounding box
        
        x_min = int(center + bb_min.x / cell_size)
        y_min = int(center + bb_min.y / cell_size)
        x_max = int(center + bb_max.x / cell_size)
        y_max = int(center + bb_max.y / cell_size)
        
        # Skip if outside grid boundaries
        if x_min < 0 or x_max >= grid_size or y_min < 0 or y_max >= grid_size:
            continue
        
        # Draw bounding box as a rectangle (blue for static objects)
        cv2.rectangle(grid, (x_min, y_min), (x_max, y_max), (255, 0, 0), thickness=1)
    
    return grid

client = carla.Client('localhost', 2000)
world = client.get_world()
grid = create_2d_grid(world)
cv2.imwrite('grid.png', grid)

def main():
    try:
        # Connect to CARLA server
        client = carla.Client('localhost', 2000)
        # client.set_timeout(2.0)
        
        world = client.get_world()
        
        while True:
            # Create and display a 2D grid with bounding boxes
            grid = create_2d_grid(world)
            cv2.imshow('CARLA Bounding Boxes', grid)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
