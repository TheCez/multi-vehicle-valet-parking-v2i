import numpy as np
import carla
import cv2
import random
import time
import subprocess
import threading

class OccupationGrid:
    def __init__(self, world, grid_size=500, cell_size=1):
        self.world = world
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.center = grid_size // 2
        self.grid = self.create_2d_obstacle_grid()

    def create_2d_obstacle_grid(self):
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        center = self.center
        walls = self.world.get_level_bbs(carla.CityObjectLabel.Other)
        for bb in walls:
            self.mark_bounding_box(grid, bb, center, self.cell_size, value=1)
        return grid

    def mark_ego_vehicle(self, grid, ego_vehicle):
        new_grid = grid.copy()
        ego_bb = ego_vehicle.bounding_box
        ego_transform = ego_vehicle.get_transform()
        self.mark_bounding_box(new_grid, ego_bb, self.center, self.cell_size, value=2, transform=ego_transform)
        return new_grid

    def mark_bounding_box(self, grid, bb, center, cell_size, value, transform=None):
        if transform:
            bb_center = transform.transform(bb.location)
            rotation = transform.rotation
        else:
            bb_center = bb.location
            rotation = bb.rotation

        bb_extent = bb.extent
        corners = self.get_bounding_box_corners(bb_center, bb_extent, rotation)
        grid_corners = []
        for corner in corners:
            x = int(center + corner.x / cell_size)
            y = int(center + corner.y / cell_size)
            grid_corners.append((x, y))
        mask = np.zeros(grid.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(grid_corners, dtype=np.int32)], value)
        grid[mask == value] = value

    def get_bounding_box_corners(self, center, extent, rotation):
        yaw_rad = np.radians(rotation.yaw)
        local_corners = [
            carla.Location(x=-extent.x, y=-extent.y),
            carla.Location(x= extent.x, y=-extent.y),
            carla.Location(x= extent.x, y= extent.y),
            carla.Location(x=-extent.x, y= extent.y),
        ]
        world_corners = []
        for corner in local_corners:
            rotated_x = corner.x * np.cos(yaw_rad) - corner.y * np.sin(yaw_rad)
            rotated_y = corner.x * np.sin(yaw_rad) + corner.y * np.cos(yaw_rad)
            world_corner = carla.Location(
                x=center.x + rotated_x,
                y=center.y + rotated_y,
                z=center.z
            )
            world_corners.append(world_corner)
        return world_corners
    
    def draw_polygons_on_grid(self,
                            target_grid,
                            polygons,
                            color=(0, 255, 0),
                            thickness=2):
        """
        Draws CommonRoad polygons onto the full grid using their absolute world coordinates.
        This logic now perfectly mirrors the mark_bounding_box function.

        Args:
            target_grid: The full, color grid image to draw on.
            polygons: A list of polygons with vertices in CommonRoad coordinates.
            color: The color for the polygon lines.
            thickness: The thickness of the polygon lines.
        """
        if not polygons:
            return target_grid

        for polygon in polygons:
            pts_on_grid = []
            for cr_x, cr_y in polygon:
                # 1. Convert CommonRoad vertex to absolute CARLA world coordinates.
                # This is the only transformation needed for the vertices themselves.
                world_x = cr_x
                world_y = -cr_y

                # 2. Map the absolute world coordinates to grid pixel coordinates.
                # This logic is now identical to the mapping in mark_bounding_box.
                grid_col = int(self.center + (world_x / self.cell_size))
                grid_row = int(self.center + (world_y / self.cell_size))

                pts_on_grid.append([grid_col, grid_row])

            # 3. Draw the complete polygon onto the target grid.
            if len(pts_on_grid) >= 3:
                cv2.polylines(target_grid,
                            [np.array(pts_on_grid, dtype=np.int32)],
                            isClosed=True,
                            color=color,
                            thickness=thickness)

        return target_grid


    def start_visualization(self, window_name='Animated Obstacle Grid'):
        """
        Initializes the visualization window and color map.
        """
        self.window_name = window_name
        # Color map: 0=white (free), 1=black (obstacle), 2=red (ego vehicle)
        self.color_map = np.array([[255, 255, 255], [0, 0, 0], [255, 0, 0]], dtype=np.uint8)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 800)
        self.visualization_running = True

    

    def update_visualization(self, ego_vehicle, zoom_factor=2, context_size=200, polygons=None):
        """
        Updates the visualization with the current ego vehicle position.
        Shows a zoomed-in context around the ego vehicle if present.
        """
        # Ensure visualization is initialized
        if not hasattr(self, 'window_name'):
            self.start_visualization()
        # Mark ego vehicle on the grid
        current_grid = self.mark_ego_vehicle(self.grid, ego_vehicle)
        # Convert grid to color image
        colored_grid = self.color_map[current_grid]
        # 3. Draw polygons directly onto the full colored grid
        if polygons is not None:
            # Note: We no longer pass ego_vehicle to this function
            colored_grid = self.draw_polygons_on_grid(
                colored_grid, polygons, color=(0, 0, 0), thickness=1 # Using black for road lines
            )

        # Find ego vehicle position
        ego_positions = np.where(current_grid == 2)
        if len(ego_positions[0]) > 0:
            # Center context window around ego vehicle
            center_y, center_x = ego_positions[0].mean(), ego_positions[1].mean()
            start_y = max(0, int(center_y - context_size // 2))
            end_y = min(self.grid.shape[0], int(center_y + context_size // 2))
            start_x = max(0, int(center_x - context_size // 2))
            end_x = min(self.grid.shape[1], int(center_x + context_size // 2))
            # Adjust window if near grid edges
            if end_y >= self.grid.shape[0] - 10:
                shift = end_y - (self.grid.shape[0] - 10)
                start_y = max(0, start_y - shift)
                end_y = self.grid.shape[0]
            if start_y <= 10:
                shift = 10 - start_y
                end_y = min(self.grid.shape[0], end_y + shift)
                start_y = 0
            if end_x >= self.grid.shape[1] - 10:
                shift = end_x - (self.grid.shape[1] - 10)
                start_x = max(0, start_x - shift)
                end_x = self.grid.shape[1]
            if start_x <= 10:
                shift = 10 - start_x
                end_x = min(self.grid.shape[1], end_x + shift)
                start_x = 0
            # Extract and zoom context window
            context_grid = colored_grid[start_y:end_y, start_x:end_x]
            # Draw polygons if provided
            print("Polygons:", polygons)
        # # Draw polygons if provided
        #     if polygons is not None:
        #         ego_transform = ego_vehicle.get_transform()
        #         context_grid = self.draw_polygons_on_grid(
        #             context_grid, polygons, ego_vehicle, start_x, start_y
        #         )
            zoomed_grid = cv2.resize(context_grid, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_NEAREST)
            cv2.imshow(self.window_name, zoomed_grid)
        else:
            # Show full grid if ego vehicle not found
            cv2.imshow(self.window_name, colored_grid)
        # Stop visualization if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.visualization_running = False
        # Small delay for animation effect
        time.sleep(0.1)

    def stop_visualization(self):
        """
        Closes the visualization window and stops the animation.
        """
        cv2.destroyAllWindows()
        self.visualization_running = False

# # Main execution
# client = carla.Client('localhost', 2000)
# world = client.get_world()

# # Get all actors in the world
# all_actors = world.get_actors()

# # Filter for vehicles
# vehicles = all_actors.filter('vehicle.*')
# print(vehicles)

# ego_vehicle=vehicles[0]



# # Create the static obstacle grid
# static_obstacle_grid = create_2d_obstacle_grid(world)




# # Start visualization in a separate thread

# vis_thread = threading.Thread(target=visualize_grid_animated, args=(static_obstacle_grid, ego_vehicle, 2, 200))
# vis_thread.start()
 

