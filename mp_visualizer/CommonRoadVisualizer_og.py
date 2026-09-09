from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer
import numpy as np
import carla
from carla_getter import carla_to_commonroad_transform_actor
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.state import InitialState
from configuration_creator import real_time_reachability_analysis
from mp_visualizer.MPRendererCanvas import MPRendererCanvas
from commonroad_reach.utility.visualization import generate_default_drawing_parameters, draw_reachable_sets, compute_plot_limits_from_reachable_sets, draw_drivable_area
import seaborn as sns
import matplotlib.pyplot as plt
from PyQt6.QtCore import QObject, pyqtSignal, QMutex, QMutexLocker
from commonroad_reach.utility.coordinate_system import convert_to_cartesian_polygons
import math
import copy
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
from commonroad.visualization.draw_params import DynamicObstacleParams
import threading

class CommonRoadVisualizer(QMainWindow):
    """Class for visualizing CommonRoad scenarios with MPRenderer in a PyQt application."""
    def __init__(self, base_config, scenario, planning_problem, world, ego_vehicle=None, visualize = False):
        super().__init__()
        self.visualize = visualize
        if self.visualize:
            self.setWindowTitle("CommonRoad Visualization with MPRenderer")
        # Initialize Carla client
        self.world = world
        
        # Store scenario elements
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.base_config = base_config
        self.reach_interface = None  
        self.ego_vehicle = ego_vehicle
        if self.visualize:
            # Setup layout
            self.central_widget = QWidget()
            self.setCentralWidget(self.central_widget)
            layout = QVBoxLayout(self.central_widget)
        

        # # Remove excessive margins
        # layout.setContentsMargins(0, 0, 0, 0)  # Zero margins around layout
        # layout.setSpacing(0)  # Remove spacing between widgets

        if self.visualize:
            # Create matplotlib canvas with MPRenderer
            self.canvas = MPRendererCanvas(figsize=(12, 10))
            layout.addWidget(self.canvas)
        
        # Draw static elements initially
        self.draw_static_elements()
        
        # # Set up update timer
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update_visualization)
        # self.timer.start(100)  # 10 FPS - matplotlib is slower than pyqtgraph

    def draw_static_elements(self):
        """Draw static elements of the scenario"""
        # print(f"Type of self.canvas.mp_renderer: {type(self.canvas.mp_renderer)}")
        # from mp_visualizer.MPRendererCanvas import MPRenderer
        # if not isinstance(self.canvas.mp_renderer, MPRenderer):
        #     print("Error: self.canvas.mp_renderer is not of type MPRenderer. Stopping execution.")
        #     return
        if self.visualize:
            # Draw the scenario (includes lanelets, obstacles, etc.)
            self.scenario.draw(self.canvas.mp_renderer)
            
            # # Draw planning problem (includes initial and goal states)
            # if hasattr(self.planning_problem, 'draw'):
            #     self.planning_problem.draw(self.canvas.mp_renderer)
            # else:
            #     # Create a planning problem set if we only have a single problem
            #     planning_problem_set = PlanningProblemSet([self.planning_problem])
            #     planning_problem_set.draw(self.canvas.mp_renderer)
        
        # Render the canvas
        #self.canvas.render()

    def update_visualization(self, other_cars=None):
        """Update dynamic elements (ego vehicle and reachability analysis)"""

        if self.visualize:

            # with QMutexLocker(self.mutex):
            # Clear previous visualization
            self.canvas.clear()
            
            # Re-draw static elements
            self.draw_static_elements()
        polygons = []  # Ensure polygons is always defined
        
        try:
            # Get latest CARLA state
            # ego_vehicle = self.world.get_actors().filter('vehicle.*')[0]
            position, orientation = carla_to_commonroad_transform_actor(self.ego_vehicle)
            angular_velocity = self.ego_vehicle.get_angular_velocity()
            velocity = self.ego_vehicle.get_velocity()
            v_x = velocity.x
            v_y = velocity.y
            speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # Convert to speed
            slip_angle = math.atan2(v_y, v_x)
            slip_angle = np.degrees(slip_angle)  # Convert to degrees
            
            # Update the planning problem's initial state
            initial_state = InitialState(
                position=position,
                orientation=orientation,
                velocity=speed * 3.6,  # Convert m/s to km/h
                # velocity = 10,
                time_step=0,
                yaw_rate=angular_velocity.z,
                slip_angle=slip_angle,
            )
            self.planning_problem.initial_state = initial_state
            
            # # Perform reachability analysis
            # self.reach_interface = real_time_reachability_analysis(
            #     self.base_config, self.scenario, self.planning_problem
            # )



            # # Draw reachable sets if available
            # if self.reach_interface:
            #     try:
            #         polygons = self.draw_reachable_area(self.base_config.planning.steps_computation, self.reach_interface)
            #     except Exception as e:
            #         print(f"Error in draw_reachable_area: {e}")
            #         polygons = []

            decision_initial_state =InitialState(
                position=position,
                orientation=orientation,
                #velocity=speed * 3.6,  # Convert m/s to km/h
                velocity = 7,
                time_step=0,
                yaw_rate=angular_velocity.z,
                slip_angle=slip_angle,
            )

            # decision_scenario = copy.deepcopy(self.scenario)
            decision_scenario = copy.copy(self.scenario)
            # if other_cars is not None:
            #     # Create a list to hold obstacles for other vehicles
            #     obstacles_to_add = []
            #     for car in other_cars:
            #         position, orientation = carla_to_commonroad_transform_actor(car)
            #         velocity = car.get_velocity()
            #         speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            #         angular_velocity = car.get_angular_velocity()
            #         slip_angle = math.atan2(velocity.y, velocity.x)
            #         slip_angle = np.degrees(slip_angle)  # Convert to degrees

            #         car_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
            #         car_initial_state = InitialState(
            #             position=position,
            #             orientation=orientation,
            #             velocity=speed * 3.6,  # Convert m/s to km/h
            #             #velocity=20,
            #             time_step=0,
            #             yaw_rate=angular_velocity.z,
            #             slip_angle=slip_angle,
            #         )
                    
            #         # Add as static obstacle instead of dynamic
            #         car_static_obstacle = StaticObstacle(
            #             obstacle_id=decision_scenario.generate_object_id(),
            #             obstacle_type=ObstacleType.CAR,
            #             obstacle_shape=car_rect,
            #             initial_state=car_initial_state
            #         )
            #         obstacles_to_add.append(car_static_obstacle)
            #         if self.visualize:
            #             car_draw_params = DynamicObstacleParams()
            #             car_draw_params.facecolor = 'blue'
            #             car_static_obstacle.draw(self.canvas.mp_renderer, draw_params=car_draw_params)
            #     # Add all obstacles at once
            #     decision_scenario.add_objects(obstacles_to_add)

            decision_planning_problem = copy.deepcopy(self.planning_problem)
            # decision_planning_problem = copy.copy(self.planning_problem)
            decision_planning_problem.initial_state = decision_initial_state
            decision_base_config = copy.deepcopy(self.base_config)
            # decision_base_config = copy.copy(self.base_config)
            decision_base_config.planning.steps_computation = 12
            # decision_interface = real_time_reachability_analysis(
            #     decision_base_config, decision_scenario, decision_planning_problem
            # )

            if self.visualize:
            
                # Draw ego vehicle

                
                ego_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
                ego_obstacle = DynamicObstacle(
                    obstacle_id=100000,
                    obstacle_type=ObstacleType.CAR,
                    obstacle_shape=ego_rect,
                    initial_state=initial_state
                )

                # Create a proper DrawParams object for dynamic obstacles

                draw_params = DynamicObstacleParams()
                draw_params.facecolor = 'red'  # Set the color property
                
                # Draw the ego vehicle with custom styling
                ego_obstacle.draw(self.canvas.mp_renderer, draw_params=draw_params)
            
            # # Draw reachable sets if available
            # if self.reach_interface:
            #     try:
            #         polygons = self.draw_reachable_area(self.base_config.planning.steps_computation, self.reach_interface)
            #     except Exception as e:
            #         print(f"Error in draw_reachable_area: {e}")
            #         polygons = []


#######################################################################################################################

            if other_cars is not None:
                # Create a list to hold obstacles for other vehicles
                obstacles_to_add = []
                for car in other_cars:
                    position, orientation = carla_to_commonroad_transform_actor(car)
                    velocity = car.get_velocity()
                    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                    angular_velocity = car.get_angular_velocity()
                    slip_angle = math.atan2(velocity.y, velocity.x)
                    slip_angle = np.degrees(slip_angle)  # Convert to degrees

                    car_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
                    car_initial_state = InitialState(
                        position=position,
                        orientation=orientation,
                        velocity=speed * 3.6,  # Convert m/s to km/h
                        #velocity=20,
                        time_step=0,
                        yaw_rate=angular_velocity.z,
                        slip_angle=slip_angle,
                    )
                    
                    # Add as static obstacle instead of dynamic
                    car_static_obstacle = StaticObstacle(
                        obstacle_id=self.scenario.generate_object_id(),
                        obstacle_type=ObstacleType.CAR,
                        obstacle_shape=car_rect,
                        initial_state=car_initial_state
                    )
                    obstacles_to_add.append(car_static_obstacle)
                    if self.visualize:
                        car_draw_params = DynamicObstacleParams()
                        car_draw_params.facecolor = 'blue'
                        car_static_obstacle.draw(self.canvas.mp_renderer, draw_params=car_draw_params)
                # Add all obstacles at once
                self.scenario.add_objects(obstacles_to_add)
########################################################################################################################


            # Prepare containers for results

            reachability_results = {}

            def run_reachability(key, config, scenario, planning_problem):
                try:
                    interface = real_time_reachability_analysis(config, scenario, planning_problem)
                    reachability_results[key] = interface
                except Exception as e:
                    print(f"Error in real_time_reachability_analysis ({key}): {e}")
                    reachability_results[key] = None

            # Start threads for both reachability analyses
            threads = []
            threads.append(threading.Thread(target=run_reachability, args=(
                "main", self.base_config, self.scenario, self.planning_problem)))
            threads.append(threading.Thread(target=run_reachability, args=(
                "decision", decision_base_config, decision_scenario, decision_planning_problem)))

            for t in threads:
                t.start()
            for t in threads:
                t.join()

            
            if 'obstacles_to_add' in locals():
                self.scenario.remove_obstacle(obstacles_to_add)
                # for obs in obstacles_to_add:
                #     self.scenario.remove_obstacle(obs)

            # Draw reachable sets if available
            polygons = []
            if reachability_results.get("main"):
                try:
                    polygons = self.draw_reachable_area(self.base_config.planning.steps_computation, reachability_results["main"])
                except Exception as e:
                    print(f"Error in draw_reachable_area (main): {e}")
                    polygons = []

            decision_polygons = []
            if reachability_results.get("decision"):
                try:
                    decision_polygons = self.draw_reachable_area(decision_base_config.planning.steps_computation, reachability_results["decision"])
                except Exception as e:
                    print(f"Error in draw_reachable_area (decision): {e}")
                    decision_polygons = []

        except Exception as e:
            print(f"Error in update_visualization: {e}")
        
        if self.visualize: 
            # Render the updated visualization
            self.canvas.render()
        # Return polygons for further processing if needed
        return polygons, decision_polygons
    
    def get_reachable_polygons(self, current_step, reach_interface):
        """Get reachable set polygons for the current time step"""

        # Get reachable set nodes
        list_nodes = reach_interface.reachable_set_at_step(current_step)

        # Convert reachable set rectangles to polygons and return them

        # clcs = reach_interface.config.planning.CLCS
        polygons = []
        for node in list_nodes:
            vertices = node.position_rectangle.vertices
            polygons.append(vertices)
        print(f"Number of polygons in reachable set: {len(polygons)}")
        # list_nodes = reach_interface.get_reachable_set_at_time_step(current_step)
        # polygons = []
        # if list_nodes:
        #     for node in list_nodes:
        #         if node.reachable_set is not None:
        #             cartesian_polygons = convert_to_cartesian_polygons(
        #                 node.reachable_set, reach_interface.config.scenario.lanelet_network
        #             )
        #             polygons.extend(cartesian_polygons)
        return polygons, list_nodes
    

    def draw_reachable_area(self, current_step, reach_interface=None):
        """Draw the reachable area for the current time step"""

        # Get reachable set nodes
        list_nodes = reach_interface.reachable_set_at_step(current_step)

        # Convert reachable set rectangles to polygons and return them

        # clcs = reach_interface.config.planning.CLCS
        # Use list comprehension for faster execution
        polygons = [node.position_rectangle.vertices for node in list_nodes]
        print(f"Number of polygons in reachable set: {len(polygons)}")


        if self.visualize:
            # generate default drawing parameters
            config = reach_interface.config
            draw_params = generate_default_drawing_parameters(config)
            palette = sns.color_palette("GnBu_d", 3)
            edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
            draw_params.shape.facecolor = palette[0]
            draw_params.shape.edgecolor = edge_color
            draw_reachable_sets(list_nodes, config, self.canvas.mp_renderer, draw_params)

        # # Get reachable set nodes
        # list_nodes = reach_interface.reachable_set_at_step(current_step)

        # # Convert reachable set rectangles to polygons and return them

        # # clcs = reach_interface.config.planning.CLCS
        # polygons = []
        # for node in list_nodes:
        #     vertices = node.position_rectangle.vertices
        #     polygons.append(vertices)
        # print(f"Number of polygons in reachable set: {len(polygons)}")

        return polygons