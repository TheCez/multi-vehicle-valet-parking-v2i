import numpy as np



def solve_conflict(conflict_area):
    """
    This function is a placeholder for the conflict resolution logic.
    It should contain the logic to resolve conflicts in the CommonRoad scenarios.
    """
    # Implement conflict resolution logic here
    conflict_area[(conflict_area == 3) | (conflict_area == -2)] = 0

    #conflict_area[conflict_area == 2] = 1
    start_path_finding =False

    # Create a flag array to mark positions where a 2 is surrounded by at least four 5s
    rows, cols = conflict_area.shape

    # Find all indices where conflict_area == 2
    twos_indices = np.argwhere(conflict_area == 2)

    for row, col in twos_indices:
        # Get 8-connected neighbors' indices
        neighbors = [
            (row-1, col-1), (row-1, col), (row-1, col+1),
            (row, col-1),               (row, col+1),
            (row+1, col-1), (row+1, col), (row+1, col+1)
        ]
        five_count = 0
        for r, c in neighbors:
            if 0 <= r < rows and 0 <= c < cols:
                if conflict_area[r, c] == 5:
                    five_count += 1
        if five_count >= 4:
            start_path_finding = True


    waypoint = None
            

    if start_path_finding:
        conflict_area[conflict_area == 2] = 1
        # Find indices where conflict_area == -4
        neg4_indices = np.argwhere(conflict_area == -4)

        for neg4_idx in neg4_indices:
            row, col = neg4_idx
            # Get 8-connected neighbors' indices
            neighbors = [
                (row-1, col-1), (row-1, col), (row-1, col+1),
                (row, col-1),               (row, col+1),
                (row+1, col-1), (row+1, col), (row+1, col+1)
            ]
            # Check if exactly 3 neighbors are either 0 or 2, and the remaining 5 are 5
            zero_or_two_count = 0
            five_count = 0
            for r, c in neighbors:
                if 0 <= r < conflict_area.shape[0] and 0 <= c < conflict_area.shape[1]:
                    val = conflict_area[r, c]
                    if val in [0, 2]:
                        zero_or_two_count += 1
                    elif val == 5:
                        five_count += 1
            if zero_or_two_count == 2 and five_count == 4:
                waypoint = (row, col)
        conflict_area[(conflict_area == -4) | (conflict_area == 5)] = 0
        
    return conflict_area , waypoint if start_path_finding else None
    
    #pass
