import numpy as np



def solve_conflict(conflict_area):
    """
    This function is a placeholder for the conflict resolution logic.
    It should contain the logic to resolve conflicts in the CommonRoad scenarios.
    """
    # Implement conflict resolution logic here
    #conflict_area[(conflict_area == 3) | (conflict_area == -2) | (conflict_area == 6)  | (conflict_area == 5)] = 0
    #conflict_area[conflict_area == 2] = 1

    waypoint = None
    start_path_finding = True
            

    if start_path_finding:
        conflict_area[conflict_area == 3] = 0
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
            two_count = 0
            for r, c in neighbors:
                if 0 <= r < conflict_area.shape[0] and 0 <= c < conflict_area.shape[1]:
                    val = conflict_area[r, c]
                    if val == 0:
                        zero_count += 1
                    elif val == 2:
                        two_count += 1
            if zero_count == 4 and two_count == 2:
                #conflict_area[row, col] = 3
                waypoint = (row, col)
        #print("Waypoint found at:", waypoint)
        waypoint = (waypoint[0] + 5, waypoint[1])
        #conflict_area[waypoint[0] + 5, waypoint[1]] = 3
        conflict_area[(conflict_area == -4)] = 0
        conflict_area[ (conflict_area == -2) 
                      #| (conflict_area == 6)  
                      | (conflict_area == 5)] = 0
        conflict_area[conflict_area == 2] = 1
        conflict_area[waypoint[0], waypoint[1]] = 3


        
    return conflict_area , waypoint
    
    #pass
