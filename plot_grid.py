import numpy as np
from scipy.ndimage import label
import matplotlib.pyplot as plt

def check_if_twos_fit_in_threes(grid):
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


# Load the 2D grid
#grid = np.load('final_grid.npy')

#grid = np.load('final_grid.npy', allow_pickle=True)
#grid = np.load('conflict/conflict_area6.npy', allow_pickle=True)
#grid = np.load('decision_grids/decision_grids_e11f01db-1882-41a5-8e1e-348c5b42c17b.npy', allow_pickle=True)
#grid = np.load('decision_grids/decision_grids_2bdcfb9c-2167-4b3b-a5a1-ef4b0bfd3fa5.npy', allow_pickle=True)

grid = np.load('decision_grids/decision_grids_378a3fbc-f038-4125-94da-914201fd0a06.npy', allow_pickle=True)

# Create the plot
plt.figure(figsize=(10, 10))
plt.imshow(grid, cmap='tab10', interpolation='nearest', vmin=-4, vmax=4)
plt.title('2D Grid Obstacle Map')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.colorbar(label='Obstacle Presence')
plt.tight_layout()
plt.savefig('grid_plot.png')
plt.show()

check_if_twos_fit_in_threes(grid)


#print(grid)

# indices = np.argwhere(grid == -2)
# #print(len(indices))
# if indices.size > 0:
#     first_idx = indices[0]
#     last_idx = indices[-1]
#     neighbors = []
#     for idx in [first_idx, last_idx]:
#         r, c = idx
#         for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
#             nr, nc = r+dr, c+dc
#             if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
#                 neighbors.append((nr, nc))
#             else:
#                 neighbors.append(None)
#     # Check which neighbor has value 2
#     #print(neighbors)
#     direction = None
#     for idx, neighbor in enumerate(neighbors):
#         if neighbor is not None:
#             nr, nc = neighbor
#             print(nr, nc, grid[nr, nc])
#             if grid[nr, nc] == 2:
#                 if (idx+1)%4 == 1:  # top neighbor
#                     print("top")
#                     direction = 'R'
#                 elif (idx+1)%4 == 2:  # bottom neighbor
#                     print("bottom")
#                     direction = 'L'
#                 elif (idx+1)%4 == 3:  # left neighbor
#                     print("left")
#                     direction = 'U'
#                 elif (idx+1)%4 == 4:  # right neighbor
#                     print("right")
#                     direction = 'D'
#                 else:
#                     print("No direction found")
#                 break

# # 1. find leftmost column index of any –2
# path_locs = np.where(grid == -2)
# if path_locs[1].size == 0:
#     raise ValueError("No -2 found in grid")
# leftmost = path_locs[1].min()

# # 2. build a mask for any 3 in columns strictly left of that
# cols = np.arange(grid.shape[1])[None, :]    # shape (1, W)
# mask = (grid == 3) & (cols < leftmost)

# # 3. zero out those positions
# grid[mask] = 0



# Create the plot
plt.figure(figsize=(10, 10))
plt.imshow(grid, cmap='tab10', interpolation='nearest', vmin=-4, vmax=4)
plt.title('2D Grid Obstacle Map')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.colorbar(label='Obstacle Presence')
plt.tight_layout()
plt.savefig('grid_plot.png')
plt.show()
