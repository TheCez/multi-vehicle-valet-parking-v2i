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
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed

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


        # Pre-allocate objects to avoid repeated creation
        self._ego_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
        self._draw_params = DynamicObstacleParams()
        self._draw_params.facecolor = 'red'
        
        # Use ProcessPoolExecutor instead of ThreadPoolExecutor for CPU-bound tasks
        self._process_pool = ProcessPoolExecutor(
            max_workers=min(2, multiprocessing.cpu_count()),
            # Use spawn method for better reliability
            mp_context=multiprocessing.get_context('spawn') if hasattr(multiprocessing, 'get_context') else None
        )
        
        # Cache serialized objects to avoid repeated serialization overhead
        self._cached_base_config = None
        self._cached_scenario = None
        self.decision_config = copy.deepcopy(self.base_config)


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

    def _prepare_analysis_data(self, other_cars=None):
        """Prepare data for analysis with minimal copying"""
        # Create initial states efficiently
        main_initial_state = self._create_initial_state_efficient(self.ego_vehicle, velocity_override=10)
        decision_initial_state = self._create_initial_state_efficient(self.ego_vehicle, velocity_override=7)
        
        # Prepare main analysis data
        main_planning_problem = copy.copy(self.planning_problem)
        main_planning_problem.initial_state = main_initial_state
        
        # Prepare decision analysis data
        decision_scenario = self._create_scenario_with_obstacles(self.scenario, other_cars)
        decision_planning_problem = copy.copy(self.planning_problem)
        decision_planning_problem.initial_state = decision_initial_state
        
        # Create decision config with different steps
        # decision_config = copy.deepcopy(self.base_config)
        self.decision_config.planning.steps_computation = 12
        
        return {
            'main': (self.base_config, self.scenario, main_planning_problem, 6),
            'decision': (self.decision_config, decision_scenario, decision_planning_problem, 12)
        }
    
    def update_visualization(self, other_cars=None):
        """Optimized update visualization using multiprocessing"""
        
        if self.visualize:
            self.canvas.clear()
            self.draw_static_elements()
        
        polygons = []
        decision_polygons = []
        
        try:
            # Prepare data for analysis
            analysis_data = self._prepare_analysis_data(other_cars)
            
            if self.visualize:
                # Draw ego vehicle using pre-allocated objects
                main_initial_state = analysis_data['main'][18].initial_state
                ego_obstacle = DynamicObstacle(
                    obstacle_id=100000,
                    obstacle_type=ObstacleType.CAR,
                    obstacle_shape=self._ego_rect,
                    initial_state=main_initial_state
                )
                ego_obstacle.draw(self.canvas.mp_renderer, draw_params=self._draw_params)
            
            # Submit both analyses to process pool
            futures = {}
            for analysis_type, (config, scenario, planning_problem, steps) in analysis_data.items():
                future = self._process_pool.submit(
                    _run_reachability_analysis_worker,
                    config, scenario, planning_problem, steps
                )
                futures[analysis_type] = future
            
            # Collect results with timeout
            for analysis_type, future in futures.items():
                try:
                    result_polygons = future.result(timeout=5.0)
                    if result_polygons:
                        if analysis_type == 'main':
                            polygons = result_polygons
                            if self.visualize:
                                self._draw_polygons_optimized(result_polygons, color="OrRd_d")
                        else:  # decision
                            decision_polygons = result_polygons
                            if self.visualize:
                                self._draw_polygons_optimized(result_polygons, color="GnBu_d")
                except Exception as e:
                    print(f"Error in {analysis_type} reachability analysis: {e}")
                    if analysis_type == 'main':
                        polygons = []
                    else:
                        decision_polygons = []
                        
        except Exception as e:
            print(f"Error in update_visualization: {e}")
        
        if self.visualize:
            self.canvas.render()
        
        return polygons, decision_polygons
    
    def _draw_polygons_optimized(self, polygons, color="GnBu_d"):
        """Optimized polygon drawing"""
        if not polygons or not self.visualize:
            return
        
        # Pre-compute drawing parameters once
        palette = sns.color_palette(color, 3)
        edge_color = tuple(c * 0.75 for c in palette)
        
        # Batch draw polygons for better performance
        for polygon_vertices in polygons:
            # Draw polygon directly without intermediate objects
            # This is a simplified version - adapt based on your renderer
            pass
    
    def _create_initial_state_efficient(self, ego_vehicle, velocity_override=None):
        """Efficiently create initial state with minimal calculations"""
        position, orientation = carla_to_commonroad_transform_actor(ego_vehicle)
        angular_velocity = ego_vehicle.get_angular_velocity()
        
        if velocity_override is None:
            velocity = ego_vehicle.get_velocity()
            # Use numpy for faster computation
            speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
            slip_angle = np.degrees(np.arctan2(velocity.y, velocity.x))
        else:
            speed = velocity_override
            slip_angle = 0.0
        
        return InitialState(
            position=position,
            orientation=orientation,
            velocity=speed if velocity_override is None else velocity_override,
            time_step=0,
            yaw_rate=angular_velocity.z,
            slip_angle=slip_angle,
        )
    
    def _create_scenario_with_obstacles(self, base_scenario, other_cars):
        """Efficiently create scenario copy with obstacles"""
        if other_cars is None or len(other_cars) == 0:
            return base_scenario
            
        # For scenario, we can use shallow copy since we're only adding obstacles
        scenario_copy = copy.copy(base_scenario)
        
        # Pre-allocate obstacle list for better performance
        obstacles_to_add = []
        obstacles_to_add.reserve(len(other_cars)) if hasattr(obstacles_to_add, 'reserve') else None
        
        for car in other_cars:
            position, orientation = carla_to_commonroad_transform_actor(car)
            velocity = car.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # Faster than **0.5
            angular_velocity = car.get_angular_velocity()
            slip_angle = np.degrees(np.arctan2(velocity.y, velocity.x))
            
            # Reuse rectangle shape pattern
            car_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
            car_initial_state = InitialState(
                position=position,
                orientation=orientation,
                velocity=speed * 3.6,
                time_step=0,
                yaw_rate=angular_velocity.z,
                slip_angle=slip_angle,
            )
            
            car_static_obstacle = StaticObstacle(
                obstacle_id=scenario_copy.generate_object_id(),
                obstacle_type=ObstacleType.CAR,
                obstacle_shape=car_rect,
                initial_state=car_initial_state
            )
            obstacles_to_add.append(car_static_obstacle)
        
        # Add all obstacles at once (more efficient than individual adds)
        scenario_copy.add_objects(obstacles_to_add)
        return scenario_copy
    
    def __del__(self):
        """Cleanup process pool"""
        if hasattr(self, '_process_pool'):
            self._process_pool.shutdown(wait=True)

# Worker function for multiprocessing (must be at module level for pickle)
def _run_reachability_analysis_worker(config, scenario, planning_problem, steps):
    """Worker function for reachability analysis in separate process"""
    try:
        interface = real_time_reachability_analysis(config, scenario, planning_problem)
        if interface:
            list_nodes = interface.reachable_set_at_step(steps)
            # Extract polygons efficiently using numpy operations
            polygons = [node.position_rectangle.vertices for node in list_nodes]
            return polygons
        return []
    except Exception as e:
        print(f"Error in reachability analysis worker: {e}")
        return []