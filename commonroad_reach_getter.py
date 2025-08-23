from commonroad_reach import ReachableSetComputation

from commonroad.common.file_reader import CommonRoadFileReader

# Load the converted scenario
scenario_path = "DEU_valetparking-1_1_T-1.xml"
scenario, _ = CommonRoadFileReader(scenario_path).open()


# Initialize with scenario and planning problem
reach_computer = ReachableSetComputation(
    scenario=scenario,
    planning_problem=planning_problem,
    vehicle_params=VehicleParameters()
)