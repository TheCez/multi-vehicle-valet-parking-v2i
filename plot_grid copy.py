import numpy as np
import matplotlib.pyplot as plt

# Load the 2D grid
#grid = np.load('final_grid.npy')

#grid = np.load('final_grid.npy', allow_pickle=True)
grid = np.load('conflict/conflict_area6.npy', allow_pickle=True)

print(grid)

# Convert grid of lists to a 2D array showing the number of overlaps at each cell
overlap_grid = np.vectorize(len)(grid)

# Create the plot
plt.figure(figsize=(10, 10))
plt.imshow(overlap_grid, cmap='tab10', interpolation='nearest', vmin=0, vmax=overlap_grid.max())
plt.title('2D Grid Overlap Map')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.colorbar(label='Number of Overlaps')
plt.tight_layout()
plt.savefig('grid_plot.png')
plt.show()
