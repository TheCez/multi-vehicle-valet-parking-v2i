from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.state import InitialState, CustomState, Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
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
import tempfile
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_parser.parser import parse_opendrive
from pathlib import Path
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_conversion.network import Network
from crdesigner.map_conversion.common.utils import generate_unique_id




class CommonRoadSceneGenerator:
    def __init__(self, world, reference_path=None):

        self.reference_path = reference_path
        # Load the converted scenario
        #scenario_path = "DEU_valetparking-1_1_T-1_base.xml"
        # scenario_path = "ZAM_MUC-1_1_T-1.xml"
        # self.scenario, _ = CommonRoadFileReader(scenario_path).open()

        self.scenario = self.scenario_loader(world)
        # Connect to the client and retrieve the world object
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()
        # Get the ego vehicle actor
        self.ego_vehicle = world.player  # Assuming ego_vehicle is passed as an argument
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
        self.base_config.planning.steps_computation = 6
        self.base_config.planning.coordinate_system = "CART"
        #self.base_config.planning.reference_path =self.reference_path
        #self.base_config.vehicle.ego.id_type_vehicle
        #self.window = CommonRoadVisualizer(self.base_config, self.scenario, self.planning_problem, self.world)

    def scenario_loader(world):
        opendrive_str = world.get_map().to_opendrive()

        # Create a temporary file to save OpenDRIVE XML string
        with tempfile.NamedTemporaryFile(suffix=".xodr", delete=False, mode='w', encoding='utf-8') as temp_file:
            temp_file.write(opendrive_str)
            temp_path = Path(temp_file.name)

        # Now parse the temporary file path
        opendrive_obj = parse_opendrive(temp_path)

        # Convert to CommonRoad
        network = Network()
        network.load_opendrive(opendrive_obj)
        scenario = network.export_commonroad_scenario()

        for road in opendrive_obj.roads:
            for road_object in road.objects:
                if getattr(road_object, 'type', '') == "parkingSpace":
                    
                    # Center position of parking space (s, t offsets plus lateral offset from tangent)
                    position, tangent, _, _ = road.plan_view.calc(
                        road_object.s, compute_curvature=False
                    )
                    
                    center_position = np.array([
                        position[0] + road_object.t * np.cos(tangent + np.pi/2),
                        position[1] + road_object.t * np.sin(tangent + np.pi/2)
                    ])
                    
                    # Half width of the parking space (width from <object>)
                    half_width = float(getattr(road_object, 'width', 0.0)) / 2

                    object_orientation = getattr(road_object, 'hdg', 0.0) + tangent
                    
                    # Compute lateral vector perpendicular to tangent (unit vector pointing left)
                    lateral_vec = np.array([
                        np.cos(object_orientation + np.pi/2),
                        np.sin(object_orientation + np.pi/2)
                    ])
                    
                    # Calculate positions for left and right parking lines
                    left_line_position = center_position + half_width * lateral_vec
                    right_line_position = center_position - half_width * lateral_vec
                    
                    length = getattr(road_object, 'validLength', 1.0)

                    # Extract marking widths inside parkingSpace, assume you parse them earlier or fallback
                    left_marking_width = 0.0
                    right_marking_width = 0.0
                    if hasattr(road_object, 'parkingSpace') and road_object.parkingSpace is not None:
                        for marking in getattr(road_object.parkingSpace, 'markingList', []):
                            side = getattr(marking, 'side', '').lower()
                            width = float(getattr(marking, 'width', 0.0))
                            if side == "left":
                                left_marking_width = width
                            elif side == "right":
                                right_marking_width = width
                    
                    # Create obstacles for left and right parking lines
                    
                    # Left parking line obstacle
                    left_line_shape = Rectangle(length=length, width=left_marking_width or 0.3)
                    left_line_state = InitialState(
                        position=left_line_position,
                        orientation=object_orientation,  # aligned with road tangent
                        time_step=0
                    )
                    left_line_obstacle = StaticObstacle(
                        obstacle_id=generate_unique_id(),
                        obstacle_type=ObstacleType.UNKNOWN,
                        obstacle_shape=left_line_shape,
                        initial_state=left_line_state
                    )
                    scenario.add_objects(left_line_obstacle)
                    
                    # Right parking line obstacle
                    right_line_shape = Rectangle(length=length, width=right_marking_width or 0.3)
                    right_line_state = InitialState(
                        position=right_line_position,
                        orientation=object_orientation,  # aligned with road tangent
                        time_step=0
                    )
                    right_line_obstacle = StaticObstacle(
                        obstacle_id=generate_unique_id(),
                        obstacle_type=ObstacleType.UNKNOWN,
                        obstacle_shape=right_line_shape,
                        initial_state=right_line_state
                    )
                    scenario.add_objects(right_line_obstacle)
        return scenario


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