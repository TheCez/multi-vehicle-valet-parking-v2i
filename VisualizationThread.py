from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtWidgets import QApplication
from mp_visualizer.CommonRoadVisualizer import CommonRoadVisualizer

class VisualizationThread(QThread):
    update_signal = pyqtSignal(object)  # For sending data to GUI

    def __init__(self, config, scenario, planning_problem, world):
        super().__init__()
        self.config = config
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.world = world

    def run(self):
        app = QApplication([])
        self.window = CommonRoadVisualizer(
            self.config, 
            self.scenario, 
            self.planning_problem, 
            self.world
        )
        self.window.show()
        app.exec()