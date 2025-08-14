import numpy as np



def solve_conflict(conflict_area):
    """
    This function is a placeholder for the conflict resolution logic.
    It should contain the logic to resolve conflicts in the CommonRoad scenarios.
    """
    # # Implement conflict resolution logic here
    # waypoint = None
    # start_path_finding = False
    # conflict_area [(conflict_area == -2) 
    #                #| (conflict_area == -4)
    #                ] = 0
    # conflict_area [(conflict_area == 2) | (conflict_area == 4)] = 0
    # #conflict_area [(conflict_area == -1) | (conflict_area == 1)] = 0

    # Implement conflict resolution logic here
    conflict_area[(conflict_area == 3) | (conflict_area == -2) | (conflict_area == 6)] = 0

    #conflict_area[conflict_area == 2] = 1
    start_path_finding =False
    stop = False

    # Create a flag array to mark positions where a 2 is surrounded by at least four 5s
    rows, cols = conflict_area.shape

    # Find all indices where conflict_area != 0 (i.e., points of conflict)
    conflict_indices = np.argwhere(conflict_area == 5)
    if conflict_indices.size > 0:
        # Get the bounds: top-left (min row, min col) and bottom-right (max row, max col)
        min_row, min_col = conflict_indices.min(axis=0)
        max_row, max_col = conflict_indices.max(axis=0)
        bounds = ((min_row, min_col), (max_row, max_col))
    else:
        bounds = None

    # Check if any conflict_area == 2 is within the bounds
    if bounds is not None:
        (min_row, min_col), (max_row, max_col) = bounds
        twos_within_bounds = np.argwhere(
            (conflict_area == 2) &
            (np.arange(rows)[:, None] >= min_row) & (np.arange(rows)[:, None] <= max_row) &
            (np.arange(cols)[None, :] >= min_col) & (np.arange(cols)[None, :] <= max_col)
        )
        if twos_within_bounds.size > 0:
            start_path_finding = True



    waypoint = None
            

    if start_path_finding:
        stop = True
        conflict_area[(conflict_area == 2) | (conflict_area == 4)] = 0
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
            zero_count = 0
            five_count = 0
            for r, c in neighbors:
                if 0 <= r < conflict_area.shape[0] and 0 <= c < conflict_area.shape[1]:
                    val = conflict_area[r, c]
                    if val == 0:
                        zero_count += 1
                    elif val == 5:
                        five_count += 1
            if zero_count >= 4 and five_count >=2:
                waypoint = (row, col)
        #conflict_area[(conflict_area == -4) | (conflict_area == 5)] = 0
        if waypoint is not None:
            min_row, min_col = bounds[0]
            max_row, max_col = bounds[1]
            sub_area = conflict_area[min_row:max_row+1, min_col:max_col+1]
            sub_area = np.where(sub_area == 5, 0, 1)
            conflict_area[min_row:max_row+1, min_col:max_col+1] = sub_area

            waypoint = (waypoint[0] + 10, waypoint[1])
            conflict_area[waypoint[0], waypoint[1]] = 4
            #waypoint = None
        
    return conflict_area , waypoint if start_path_finding else None, stop
    
    #pass
