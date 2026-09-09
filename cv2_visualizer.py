import numpy as np
import cv2
import glob
import os

# Define color map
COLOR_MAP = np.array([
    [255, 255, 255],   # 0: white (free)
    [0, 0, 0],         # 1: black (obstacle)
    [255, 0, 0],       # 2: red (ego vehicle)
    [0, 255, 0],       # 3: green (reachability sets)
    [0, 0, 255],       # 4: blue (other car)
    [255, 255, 0],     # 5: yellow (other car reachability set)
    [255, 0, 255],     # 6: magenta (new car path)
    [0, 128, 255]      # 7: orange (contrasting waypoint)
], dtype=np.uint8)

def visualize_npy_array(npy_file):
    arr = np.load(npy_file)
    # Rotate the array by 90 degrees counterclockwise
    arr = np.rot90(arr,2)
    # arr[arr == 2] = 4
    # arr[arr == 3] = 5
    # arr[arr == -2] = 6
    points_with_2 = np.argwhere(arr == 2)
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                       (0, -1),           (0, 1),
                       (1, -1),  (1, 0),  (1, 1)]
    neighbors = []
    for y, x in points_with_2:
        for dy, dx in neighbor_offsets:
            ny, nx = y + dy, x + dx
            if 0 <= ny < arr.shape[0] and 0 <= nx < arr.shape[1]:
                # Check if the current point (y, x) has the required neighbors
                # Above three: (y-1, x-1), (y-1, x), (y-1, x+1)
                # Left: (y, x-1), Right: (y, x+1)
                # Bottom three: (y+1, x-1), (y+1, x), (y+1, x+1)
                if (
                    0 <= y-1 < arr.shape[0] and 0 <= y+1 < arr.shape[0] and
                    0 <= x-1 < arr.shape[1] and 0 <= x+1 < arr.shape[1]
                ):
                    above = [arr[y-1, x-1], arr[y-1, x], arr[y-1, x+1]]
                    left = arr[y, x-1]
                    right = arr[y, x+1]
                    below = [arr[y+1, x-1], arr[y+1, x], arr[y+1, x+1]]
                    if all(val == 2 for val in above) and left == 2 and right == 2 and all(val == 0 for val in below):
                        neighbors.append((y, x))
                        arr[y, x] = 7

    # neighbors now contains the 8-neighbor coordinates for all points with value 2
    # arr = np.clip(arr, 0, len(COLOR_MAP)-1).astype(np.uint8)
    img = COLOR_MAP[arr]
    zoom_factor = 8  # Increase for more zoom
    img_zoomed = cv2.resize(img, (img.shape[1]*zoom_factor, img.shape[0]*zoom_factor), interpolation=cv2.INTER_NEAREST)
    cv2.imshow(f"Visualization - {os.path.basename(npy_file)}", img_zoomed)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    npy_file = "waypoint_finder_grid/decision_grid_copy_bb4502e6-d0f3-4b29-a393-c6fc5b0d4139.npy"  # Change to your npy file path
    visualize_npy_array(npy_file)
