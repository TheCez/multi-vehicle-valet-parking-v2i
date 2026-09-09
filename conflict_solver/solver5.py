import numpy as np



def solve_conflict(conflict_area):
    """
    This function is a placeholder for the conflict resolution logic.
    It should contain the logic to resolve conflicts in the CommonRoad scenarios.
    """
    # Implement conflict resolution logic here
    #conflict_area[(conflict_area == 3) | (conflict_area == -2) | (conflict_area == 6)  | (conflict_area == 5)] = 0
    #conflict_area[conflict_area == 2] = 1

    points_with_2 = np.argwhere(conflict_area == 2)
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                       (0, -1),           (0, 1),
                       (1, -1),  (1, 0),  (1, 1)]
    neighbors = []
    for y, x in points_with_2:
        for dy, dx in neighbor_offsets:
            ny, nx = y + dy, x + dx
            if 0 <= ny < conflict_area.shape[0] and 0 <= nx < conflict_area.shape[1]:
                # Check if the current point (y, x) has the required neighbors
                # Above three: (y-1, x-1), (y-1, x), (y-1, x+1)
                # Left: (y, x-1), Right: (y, x+1)
                # Bottom three: (y+1, x-1), (y+1, x), (y+1, x+1)
                if (
                    0 <= y-1 < conflict_area.shape[0] and 0 <= y+1 < conflict_area.shape[0] and
                    0 <= x-1 < conflict_area.shape[1] and 0 <= x+1 < conflict_area.shape[1]
                ):
                    above = [conflict_area[y-1, x-1], conflict_area[y-1, x], conflict_area[y-1, x+1]]
                    left = conflict_area[y, x-1]
                    right = conflict_area[y, x+1]
                    below = [conflict_area[y+1, x-1], conflict_area[y+1, x], conflict_area[y+1, x+1]]
                    if all(val == 2 for val in above) and left == 2 and right == 2 and all(val == 0 for val in below):
                        neighbors.append((y, x))
                        waypoint = (y, x)
                        waypoint_og = (y, x)


    if waypoint is not None:
        y, x = waypoint
        conflict_area[y, x] = 100
    return conflict_area , waypoint, waypoint_og    
    
    #pass
