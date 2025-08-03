from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.state import InitialState, CustomState, Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.scenario import Tag  # Import Tag from the appropriate module
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion  # Import PlanningProblem, PlanningProblemSet, and GoalRegion
from commonroad.common.file_writer import OverwriteExistingFile  # Import OverwriteExistingFile
# Removed invalid import. StateTrajectory will be replaced with a valid alternative.
import math
from carla import Actor
import carla
from carla_getter import carla_to_commonroad_transform_actor, carla_to_commonroad_transform_actor_manual  # Assuming this function is defined in carla_getter.py
import numpy as np
from commonroad.visualization.mp_renderer import MPRenderer
import matplotlib.pyplot as plt
import threading
import time

#visualization
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
#from visualize import CommonRoadVisualizer
# from commonroad_reach import ReachableSetComputation
# from commonroad_reach.vehicle import VehicleParameters
from configuration_creator import create_base_configuration, update_with_dynamic_scenario, real_time_reachability_analysis
from commonroad_reach.utility import visualization as util_visual
from mp_visualizer.CommonRoadVisualizer import CommonRoadVisualizer

# # Load the converted scenario
# scenario_path = "DEU_valetparking-1_1_T-1_base.xml"
# scenario, _ = CommonRoadFileReader(scenario_path).open()


# # Get vehicle co-ordinates from carla
# # Connect to the client and retrieve the world object
# client = carla.Client('localhost', 2000)
# world = client.get_world()
# # Get the ego vehicle actor
# ego_vehicle = world.get_actors().filter('vehicle.*')[0]  # Assuming the first vehicle is the ego vehicle

# ego_shape = Rectangle(length=4.3, width=1.8)

# goal_position = ([21.841997,43.342876], 89.503952)
# ego_goal_position, ego_goal_orientation = carla_to_commonroad_transform_actor_manual(goal_position)
# ego_goal_orientation = abs(ego_goal_orientation)

# position,orientation = carla_to_commonroad_transform_actor(ego_vehicle)
# initial_state = InitialState(
#         position=position,
#         orientation=orientation,
#         velocity=8.2,  # Example velocity in m/s
#         time_step=0,
#         yaw_rate=0.0,  # Example yaw rate in rad/s
#         slip_angle=0.0,  # Example slip angle in rad

#     )

# # 4. Create DynamicObstacle with converted state
# ego_obstacle = DynamicObstacle(
#     obstacle_id=scenario.generate_object_id(),
#     obstacle_type=ObstacleType.CAR,
#     initial_state=initial_state,
#     obstacle_shape=ego_shape,
#     prediction=TrajectoryPrediction(
#         trajectory=Trajectory(initial_time_step=0, state_list=[initial_state]),  # Replaced StateTrajectory with a simple list of states
#         shape=ego_shape
#     )
# )

# # 6. Make a Goal
# # Define goal position as a shape (Rectangle)
# goal_shape = Rectangle(
#     length=4,  # Length along the lane
#     width=5,   # Width across the lane
#     center=np.array(ego_goal_position, dtype=np.float64)  # Center coordinates
# )

# # Define goal state with Interval-wrapped attributes
# goal_state = CustomState(
#     position=goal_shape,  # Position uses shape directly
#     orientation=AngleInterval(-ego_goal_orientation, ego_goal_orientation),  # Allowed orientation range
#     #velocity=Interval(0.0, 1.0),  # Velocity range [0, 1] m/s
#     time_step=Interval(0, 0)  # Time window (steps 100-120)
# )

# # Create a PlanningProblem
# # Define the planning problem with the goal region
# planning_problem = PlanningProblem(
#     planning_problem_id=0,
#     initial_state=initial_state,  # From CARLA
#     goal_region=GoalRegion(state_list=[goal_state])
# )


# # # commonroad reach
# base_config = create_base_configuration()
# base_config.planning.steps_computation = 10
# #base_config.planning.coordinate_system = "CART"

# # # 7. Visualize the scenario with MPRenderer
# app = QApplication([])
# window = CommonRoadVisualizer(base_config, scenario, planning_problem, world)
# window.setGeometry(100, 100, 800, 600)
# window.show()
# # # Start application
# app.exec()


class CommonRoadSceneGenerator:
    def __init__(self, ego_vehicle, reference_path=None):

        self.reference_path = reference_path
        # Load the converted scenario
        #scenario_path = "DEU_valetparking-1_1_T-1_base.xml"
        scenario_path = "ZAM_MUC-1_1_T-1.xml"
        self.scenario, _ = CommonRoadFileReader(scenario_path).open()

        # Connect to the client and retrieve the world object
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()
        # Get the ego vehicle actor
        self.ego_vehicle = ego_vehicle  # Assuming ego_vehicle is passed as an argument
        #self.ego_vehicle = self.world.get_actors().filter('vehicle.*')[0]  # Assuming the first vehicle is the ego vehicle

        self.ego_shape = Rectangle(length=4.3, width=1.8)

        goal_position = ([21.841997,43.342876], 89.503952)
        self.ego_goal_position, self.ego_goal_orientation = carla_to_commonroad_transform_actor_manual(goal_position)
        self.ego_goal_orientation = abs(self.ego_goal_orientation)

        position, orientation = carla_to_commonroad_transform_actor(self.ego_vehicle)
        self.initial_state = InitialState(
            position=position,
            orientation=orientation,
            velocity=0,
            time_step=0,
            yaw_rate=0.0,
            slip_angle=0.0,
        )

        # Create DynamicObstacle with converted state
        self.ego_obstacle = DynamicObstacle(
            obstacle_id=self.scenario.generate_object_id(),
            obstacle_type=ObstacleType.CAR,
            initial_state=self.initial_state,
            obstacle_shape=self.ego_shape,
            prediction=TrajectoryPrediction(
                trajectory=Trajectory(initial_time_step=0, state_list=[self.initial_state]),
                shape=self.ego_shape
            )
        )

        # Define goal position as a shape (Rectangle)
        self.goal_shape = Rectangle(
            length=4,
            width=5,
            center=np.array(self.ego_goal_position, dtype=np.float64)
        )

        # Define goal state with Interval-wrapped attributes
        self.goal_state = CustomState(
            position=self.goal_shape,
            orientation=AngleInterval(-self.ego_goal_orientation, self.ego_goal_orientation),
            time_step=Interval(0, 0)
        )

        # Create a PlanningProblem
        self.planning_problem = PlanningProblem(
            planning_problem_id=0,
            initial_state=self.initial_state,
            goal_region=GoalRegion(state_list=[self.goal_state])
        )

        # Commonroad reach
        self.base_config = create_base_configuration()
        self.base_config.planning.steps_computation = 12
        self.base_config.planning.coordinate_system = "CART"
        #self.base_config.planning.reference_path =self.reference_path
        #self.base_config.vehicle.ego.id_type_vehicle
        #self.window = CommonRoadVisualizer(self.base_config, self.scenario, self.planning_problem, self.world)

    def run(self):
        # Visualize the scenario with MPRenderer in a separate thread
        def visualize():
            app = QApplication([])
            window = CommonRoadVisualizer(self.base_config, self.scenario, self.planning_problem, self.world)
            window.setGeometry(100, 100, 800, 600)
            window.show()
            app.exec()

        vis_thread = threading.Thread(target=visualize, daemon=True)
        vis_thread.start()


# test = CommonRoadSceneGenerator()
# test.run()