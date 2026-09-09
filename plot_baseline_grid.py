import numpy as np

import matplotlib.pyplot as plt

# Path to your .npy file
npy_path = 'baseline_grid/baseline_occupancygrid2.npy'  # Change this to your actual file path

# Load the numpy array
data = np.load(npy_path)

# Plot the array
plt.imshow(data, cmap='viridis', vmin=0, vmax=5)
plt.colorbar(label='Value')
plt.title('Baseline Grid')
plt.show()