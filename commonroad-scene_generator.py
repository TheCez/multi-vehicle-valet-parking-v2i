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
from visualize import CommonRoadVisualizer
# from commonroad_reach import ReachableSetComputation
# from commonroad_reach.vehicle import VehicleParameters
from configuration_creator import create_base_configuration, update_with_dynamic_scenario, real_time_reachability_analysis
from commonroad_reach.utility import visualization as util_visual

# Load the converted scenario
scenario_path = "DEU_valetparking-1_1_T-1_base.xml"
scenario, _ = CommonRoadFileReader(scenario_path).open()


# Get vehicle co-ordinates from carla
# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()
# Get the ego vehicle actor
ego_vehicle = world.get_actors().filter('vehicle.*')[0]  # Assuming the first vehicle is the ego vehicle

ego_shape = Rectangle(length=4.3, width=1.8)

goal_position = ([21.841997,43.342876], 89.503952)
ego_goal_position, ego_goal_orientation = carla_to_commonroad_transform_actor_manual(goal_position)
ego_goal_orientation = abs(ego_goal_orientation)

position,orientation = carla_to_commonroad_transform_actor(ego_vehicle)
initial_state = InitialState(
        position=position,
        orientation=orientation,
        velocity=8.2,  # Example velocity in m/s
        time_step=0,
        yaw_rate=0.0,  # Example yaw rate in rad/s
        slip_angle=0.0,  # Example slip angle in rad

    )

# Ego vehicle update function
def update_ego_state():
    #global scenario
    global initial_state
    global position
    global orientation
    global ego_obstacle

    # Get the CommonRoad position and orientation
    position,orientation = carla_to_commonroad_transform_actor(ego_vehicle)
    #print(f"Position: {position}, Orientation: {orientation}")

    # Make InitialState
    initial_state = InitialState(
            position=position,
            orientation=orientation,
            velocity=8.2,  # Example velocity in m/s
            time_step=0,
            yaw_rate=0.0,  # Example yaw rate in rad/s
            slip_angle=0.0,  # Example slip angle in rad

        )

    # 4. Create DynamicObstacle with converted state
    ego_obstacle = DynamicObstacle(
        obstacle_id=scenario.generate_object_id(),
        obstacle_type=ObstacleType.CAR,
        initial_state=initial_state,
        obstacle_shape=ego_shape,
        prediction=TrajectoryPrediction(
            trajectory=Trajectory(initial_time_step=0, state_list=[initial_state]),  # Replaced StateTrajectory with a simple list of states
            shape=ego_shape
        )
    )

    new_scenario = scenario
    # 5. Add to scenario
    new_scenario.add_objects(ego_obstacle)

    new_scenario.assign_obstacles_to_lanelets()
    return new_scenario



live_scenario = update_ego_state()
# 6. Make a Goal
# Define goal position as a shape (Rectangle)
goal_shape = Rectangle(
    length=5.0,  # Length along the lane
    width=3.0,   # Width across the lane
    center=np.array(ego_goal_position, dtype=np.float64)  # Center coordinates
)

# Define goal state with Interval-wrapped attributes
goal_state = CustomState(
    position=goal_shape,  # Position uses shape directly
    orientation=AngleInterval(-ego_goal_orientation, ego_goal_orientation),  # Allowed orientation range
    #velocity=Interval(0.0, 1.0),  # Velocity range [0, 1] m/s
    time_step=Interval(0, 0)  # Time window (steps 100-120)
)

# Create a PlanningProblem
# Define the planning problem with the goal region
planning_problem = PlanningProblem(
    planning_problem_id=0,
    initial_state=initial_state,  # From CARLA
    goal_region=GoalRegion(state_list=[goal_state])
)


# # commonroad reach
base_config = create_base_configuration()
base_config.planning.steps_computation = 10

#reach_interface = real_time_reachability_analysis(base_config, live_scenario, planning_problem)
# #util_visual.plot_scenario_with_reachable_sets(reach_interface)

#visualization
app = QApplication([])
window = CommonRoadVisualizer(base_config, scenario, planning_problem, world)
window.setGeometry(100, 100, 800, 600)
window.show()
# Start application
app.exec()





# # 7. Visualize the scenario

# plt.ion()  # Turn on interactive mode
# rnd = MPRenderer(figsize=(15, 7))

# def visualization_thread(rnd):
#     plt.show()

# # Initial draw to create the figure and axes
# live_scenario.draw(rnd)
# planning_problem.draw(rnd)
# rnd.render(show=False)
# plt.show(block=False)


# try:
#     while True:
#         start = time.time()

#         # Update scenario (e.g., update ego position from CARLA)
#         live_scenario= update_ego_state()  # Your function

#         # Redraw
#         rnd.clear()
#         live_scenario.draw(rnd)
#         planning_problem.draw(rnd)
#         plt.draw()
#         plt.pause(0.001)  # This keeps the GUI responsive

#         # Maintain ~30 FPS (33ms per frame)
#         elapsed = time.time() - start
#         sleep_time = max(0, (1.0/30) - elapsed)
#         time.sleep(sleep_time)

# except KeyboardInterrupt:
#     plt.ioff()
#     plt.close()
#     print("Visualization stopped.")

# # Initialize plot in separate thread
# rnd = MPRenderer(figsize=(15, 7))
# thread = threading.Thread(target=visualization_thread, args=(rnd,))
# thread.daemon = True
# thread.start()

# while True:
#     # Update CARLA/CommonRoad state
#     update_ego_state()  # Your state update logic
    
#     # Redraw
#     rnd.clear()
#     scenario.draw(rnd)
#     planning_problem.draw(rnd)
#     plt.draw()
#     plt.pause(0.001)



# # After creating scenario and planning_problem:
# rnd = MPRenderer(figsize=(25, 10))  # Adjust figsize as needed

# # Draw scenario elements
# live_scenario.draw(rnd)
# planning_problem.draw(rnd)  # Requires PlanningProblemSet wrapping

# # Render the plot
# rnd.render(show=True)
# plt.show()


# 6. Save the modified scenario
# Store generated scenario
# CommonRoadFileWriter(
#     scenario,
#     PlanningProblemSet([planning_problem]),
#     author="Ajay",
#     affiliation="Technical University of Braunschweig",
#     source="CARLA",
#     tags={Tag.URBAN},
# ).write_to_file(None, OverwriteExistingFile.ALWAYS)

