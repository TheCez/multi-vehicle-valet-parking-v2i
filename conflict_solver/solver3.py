import numpy as np
# ML model imports
from grid_pattern_predictor.model import GridPatternPredictor


class ConflictSolver:
    def __init__(self):
        self.predictor = GridPatternPredictor.load('grid_pattern_predictor/grid_predictor_working_augmented.pkl')
        #self.predictor = GridPatternPredictor.load('grid_pattern_predictor/grid_predictor_working_augmented_direct.pkl')

    def solve_conflict(self, conflict_area, min_r, min_c):
        """
        This function is a placeholder for the conflict resolution logic.
        It should contain the logic to resolve conflicts in the CommonRoad scenarios.
        """
        x, y = self.predictor.predict(conflict_area)
        waypoint = (x - min_r, y - min_c)
        #waypoint = (waypoint[0] + 5, waypoint[1])
        #waypoint = (waypoint[0] - min_r, waypoint[1] - min_c)
        
        return waypoint, (x, y)
