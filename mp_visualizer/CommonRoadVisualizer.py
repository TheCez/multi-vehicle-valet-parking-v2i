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

class CommonRoadVisualizer(QMainWindow):
    def __init__(self, base_config, scenario, planning_problem, world):
        super().__init__()
        self.setWindowTitle("CommonRoad Visualization with MPRenderer")
        
        # Initialize Carla client
        self.world = world
        
        # Store scenario elements
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.base_config = base_config
        self.reach_interface = None
        
        # Setup layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # # Remove excessive margins
        # layout.setContentsMargins(0, 0, 0, 0)  # Zero margins around layout
        # layout.setSpacing(0)  # Remove spacing between widgets
        
        # Create matplotlib canvas with MPRenderer
        self.canvas = MPRendererCanvas(figsize=(12, 10))
        layout.addWidget(self.canvas)
        
        # Draw static elements initially
        self.draw_static_elements()
        
        # Set up update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(100)  # 10 FPS - matplotlib is slower than pyqtgraph

    def draw_static_elements(self):
        """Draw static elements of the scenario"""
        # print(f"Type of self.canvas.mp_renderer: {type(self.canvas.mp_renderer)}")
        # from mp_visualizer.MPRendererCanvas import MPRenderer
        # if not isinstance(self.canvas.mp_renderer, MPRenderer):
        #     print("Error: self.canvas.mp_renderer is not of type MPRenderer. Stopping execution.")
        #     return
        # Draw the scenario (includes lanelets, obstacles, etc.)
        self.scenario.draw(self.canvas.mp_renderer)
        
        # Draw planning problem (includes initial and goal states)
        if hasattr(self.planning_problem, 'draw'):
            self.planning_problem.draw(self.canvas.mp_renderer)
        else:
            # Create a planning problem set if we only have a single problem
            planning_problem_set = PlanningProblemSet([self.planning_problem])
            planning_problem_set.draw(self.canvas.mp_renderer)
        
        # Render the canvas
        #self.canvas.render()

    def update_visualization(self):
        """Update dynamic elements (ego vehicle and reachability analysis)"""
        # Clear previous visualization
        self.canvas.clear()
        
        # Re-draw static elements
        self.draw_static_elements()
        
        try:
            # Get latest CARLA state
            ego_vehicle = self.world.get_actors().filter('vehicle.*')[0]
            position, orientation = carla_to_commonroad_transform_actor(ego_vehicle)
            
            # Update the planning problem's initial state
            initial_state = InitialState(
                position=position,
                orientation=orientation,
                velocity=8.2,
                time_step=0,
                yaw_rate=0.0,
                slip_angle=0.0,
            )
            self.planning_problem.initial_state = initial_state
            
            # Perform reachability analysis
            current_step = self.base_config.planning.steps_computation
            self.reach_interface = real_time_reachability_analysis(
                self.base_config, self.scenario, self.planning_problem
            )
            
            # Draw ego vehicle
            from commonroad.geometry.shape import Rectangle
            from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
            
            ego_rect = Rectangle(length=4.3, width=1.8, center=np.zeros(2))
            ego_obstacle = DynamicObstacle(
                obstacle_id=100000,
                obstacle_type=ObstacleType.CAR,
                obstacle_shape=ego_rect,
                initial_state=initial_state
            )

            # Create a proper DrawParams object for dynamic obstacles
            from commonroad.visualization.draw_params import DynamicObstacleParams
            draw_params = DynamicObstacleParams()
            draw_params.facecolor = 'red'  # Set the color property
            
            # Draw the ego vehicle with custom styling
            ego_obstacle.draw(self.canvas.mp_renderer, draw_params=draw_params)
            
            # Draw reachable sets if available
            if self.reach_interface:
                self.draw_reachable_area(current_step)
        
        except Exception as e:
            print(f"Error in update_visualization: {e}")
        
        # Render the updated visualization
        self.canvas.render()

    def draw_reachable_area(self, current_step):
        """Draw the reachable area for the current time step"""
            # generate default drawing parameters
        config = self.reach_interface.config
        draw_params = generate_default_drawing_parameters(config)
        palette = sns.color_palette("GnBu_d", 3)
        edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
        draw_params.shape.facecolor = palette[0]
        draw_params.shape.edgecolor = edge_color
        # Get drivable area
        #list_nodes = self.reach_interface.reachable_set_at_step(current_step)
        #draw_reachable_sets(list_nodes, config, self.canvas.mp_renderer, draw_params)
        list_nodes = self.reach_interface.drivable_area_at_step(current_step)
        draw_drivable_area(list_nodes, config, self.canvas.mp_renderer, draw_params)
        #self.canvas.mp_renderer.ax.autoscale(enable=True)
        #plot_limits = compute_plot_limits_from_reachable_sets(self.reach_interface)
        #self.canvas.mp_renderer.plot_limits = plot_limits


        # plt.rc("axes", axisbelow=True)
        # ax = plt.gca()
        # ax.set_aspect("equal")
        # #ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        # ax.set_xlabel(f"$s$ [m]", fontsize=28)
        # ax.set_ylabel("$d$ [m]", fontsize=28)
        # plt.margins(0, 0)

        # # Get coordinate system from reach interface
        # clcs = self.reach_interface.config.planning.CLCS
        
        # # Import utility function to convert to cartesian coordinates
        # from commonroad_reach.utility.coordinate_system import convert_to_cartesian_polygons
        
        # # Draw each rectangle in the drivable area
        # for rectangle_cvln in drivable_area:
        #     # Convert to cartesian polygons
        #     cartesian_polygons = convert_to_cartesian_polygons(
        #         rectangle_cvln, clcs, split_wrt_angle=False
        #     )
            
        #     # Draw each polygon with custom styling
        #     for poly in cartesian_polygons:
        #         poly.draw(self.canvas.mp_renderer, 
        #                 draw_params={'facecolor': 'blue', 'opacity': 0.5})
