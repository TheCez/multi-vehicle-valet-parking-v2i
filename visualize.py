from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer, QPointF
import pyqtgraph as pg
import numpy as np
import carla
from carla_getter import carla_to_commonroad_transform_actor
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion 
from commonroad.scenario.state import InitialState
from configuration_creator import create_base_configuration, update_with_dynamic_scenario, real_time_reachability_analysis

from PyQt6.QtGui import QPolygonF, QColor, QBrush
from PyQt6.QtWidgets import QGraphicsPolygonItem
from commonroad_reach.utility.visualization import draw_drivable_area
from commonroad_reach.utility.coordinate_system import convert_to_cartesian_polygons

class CommonRoadVisualizer(QMainWindow):
    def __init__(self, base_config, scenario, planning_problem, world):
        super().__init__()

        # Initialize Carla client
        self.world = world
        
        # PyQtGraph setup
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Y Position (m)')
        self.plot_widget.setLabel('bottom', 'X Position (m)')
        
        # Store scenario elements
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.ego_item = None
        
        # Initialize static elements
        self.draw_static_elements()
        
        # Set up update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(33)  # ~30 FPS
        self.base_config = base_config
        self.reach_interface = None
        self.reach_polys = []  # To keep track of drawn polygons
        
    def draw_static_elements(self):
        """Draw lane network and static obstacles once"""
        # Draw lanelets
        for lanelet in self.scenario.lanelet_network.lanelets:
            vertices = np.array([vertex for vertex in lanelet.center_vertices])
            self.plot_widget.plot(vertices[:, 0], vertices[:, 1], pen='b')
        
        # Draw goal region
        goal_shape = self.planning_problem.goal.state_list[0].position
        goal_rect = pg.QtWidgets.QGraphicsRectItem(
            goal_shape.center[0] - goal_shape.length/2,
            goal_shape.center[1] - goal_shape.width/2,
            goal_shape.length,
            goal_shape.width
        )
        goal_rect.setBrush(pg.mkBrush(0, 255, 0, 100))
        self.plot_widget.addItem(goal_rect)

    
    def transform_to_global_coordinates(self, lon_lat_coords, reference_path):
        """
        Transform coordinates from longitudinal/lateral to global X/Y coordinates
        
        Args:
            lon_lat_coords: List of (lon, lat) coordinates 
            reference_path: Reference path object from CommonRoad
            
        Returns:
            List of (x, y) coordinates in global space
        """
        global_coords = []
        
        for lon, lat in lon_lat_coords:
            # Find position along reference path at longitudinal position 'lon'
            ref_point = reference_path.get_position(lon)
            
            # Get tangent vector (normalized) at this position
            tangent = reference_path.get_tangent(lon)
            
            # Normal vector is perpendicular to tangent
            normal = np.array([-tangent[1], tangent[0]])
            
            # Compute global position (reference point + lateral offset)
            global_point = ref_point + lat * normal
            global_coords.append(global_point)
        
        return global_coords

    


    def update_visualization(self):
        """Update dynamic elements (ego vehicle)"""
        # Get latest CARLA state
        ego_vehicle = self.world.get_actors().filter('vehicle.*')[0]
        #position = ego_vehicle.get_transform().location
        #orientation = ego_vehicle.get_transform().rotation.yaw
        position, orientation = carla_to_commonroad_transform_actor(ego_vehicle)

        # Update or create ego vehicle rectangle
        if self.ego_item is None:
            self.ego_item = pg.QtWidgets.QGraphicsRectItem(-2.15, -0.9, 4.3, 1.8)
            self.ego_item.setBrush(pg.mkBrush(255, 0, 0, 200))
            self.plot_widget.addItem(self.ego_item)
            
        # Set position and rotation
        self.ego_item.setPos(position[0], position[1])
        self.ego_item.setRotation(-np.degrees(orientation))  # Convert to degrees

        initial_state = InitialState(
        position=position,
        orientation=orientation,
        velocity=8.2,  # Example velocity in m/s
        time_step=0,
        yaw_rate=0.0,  # Example yaw rate in rad/s
        slip_angle=0.0,  # Example slip angle in rad
        )
        self.planning_problem.initial_state = initial_state
        # commonroad reach
        current_step = 10
        #base_config = create_base_configuration()

        self.reach_interface = real_time_reachability_analysis(self.base_config, self.scenario, self.planning_problem)
        #reachable_polygons = self.reach_interface.reachable_set_at_step(current_step)

        # Remove old reachability polygons
        for poly_item in self.reach_polys:
            self.plot_widget.removeItem(poly_item)
        self.reach_polys.clear()

        #current_step = ...  # your current time step
        drivable_area = self.reach_interface.drivable_area_at_step(current_step)

        # Get CLCS from your config
        clcs = self.reach_interface.config.planning.CLCS

        for rectangle_cvln in drivable_area:
            # Convert to cartesian polygons
            cartesian_polygons = convert_to_cartesian_polygons(rectangle_cvln, clcs, split_wrt_angle=False)
            for poly in cartesian_polygons:
                coords = poly.vertices

                qpoly = QPolygonF([QPointF(x, y) for x, y in coords])
                poly_item = QGraphicsPolygonItem(qpoly)
                poly_item.setBrush(QBrush(QColor(255, 0, 0, 80)))  # semi-transparent red
                poly_item.setPen(pg.mkPen('b', width=1))
                self.plot_widget.addItem(poly_item)
                self.reach_polys.append(poly_item)













        #######################################################################################

        # # Remove old reachability polygons
        # for poly_item in self.reach_polys:
        #     self.plot_widget.removeItem(poly_item)
        # self.reach_polys.clear()

        # # Draw new reachable set polygons for current step
        # #current_step = ...  # Set to your current time step
        # reachable_rectangles =  self.reach_interface.drivable_area_at_step(end_step)
        # for rect in reachable_rectangles:
        #     # If rect is a shapely Polygon or similar, get its exterior coordinates
        #     if hasattr(rect, "exterior"):
        #         coords = list(rect.exterior.coords)
        #     # If rect is a custom rectangle object with corners
        #     elif hasattr(rect, "vertices"):
        #         coords = rect.vertices
        #     # If rect has bounds (minx, miny, maxx, maxy)
        #     elif hasattr(rect, "bounds"):
        #         minx, miny, maxx, maxy = rect.bounds
        #         coords = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
        #     else:
        #         continue  # unknown format

        #     qpoly = QPolygonF([QPointF(x, y) for x, y in coords])
        #     poly_item = QGraphicsPolygonItem(qpoly)
        #     poly_item.setBrush(QBrush(QColor(0, 0, 255, 80)))  # semi-transparent blue
        #     poly_item.setPen(pg.mkPen('b', width=1))
        #     self.plot_widget.addItem(poly_item)
        #     self.reach_polys.append(poly_item)








# # Initialization
# app = QApplication([])

# # Load your scenario and planning problem
# scenario, _ = CommonRoadFileReader("DEU_valetparking-1_1_T-1_base.xml").open()
# planning_problem = ...  # Your planning problem setup

# # Create and show window
# window = CommonRoadVisualizer(scenario, planning_problem)
# window.setGeometry(100, 100, 800, 600)
# window.show()

# # Start CARLA connection
# client = carla.Client('localhost', 2000)
# world = client.get_world()

# # Start application
# app.exec()
