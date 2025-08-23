import os
from omegaconf import OmegaConf
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
import commonroad_reach.utility.logger as util_logger

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual


import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

from commonroad_reach.utility import visualization as util_visual

# Create a base configuration with default values
def create_base_configuration():
    # ==== specify scenario
    name_scenario = "valet_parking"

    # ==== build configuration
    config = ConfigurationBuilder(path_root="..").build_configuration(name_scenario)
    
    return config

def update_with_dynamic_scenario(config, scenario, planning_problem):
    """
    Update configuration with a dynamically generated scenario and planning problem
    
    Args:
        config: The baseline Configuration object
        scenario: A CommonRoad Scenario object
        planning_problem: A CommonRoad PlanningProblem object
    
    Returns:
        Updated configuration
    """
    # Update configuration with dynamic scenario and planning problem
    config.update(
        scenario=scenario,
        planning_problem=planning_problem,
        idx_planning_problem=0
    )
    
    return config

def real_time_reachability_analysis(base_config, scenario, planning_problem):
    """
    Perform real-time reachability analysis on a dynamically generated scenario
    
    Args:
        scenario: A CommonRoad Scenario object
        planning_problem: A CommonRoad PlanningProblem object
        
    Returns:
        ReachableSetInterface object with computed reachable sets
    """
    # Create base configuration
    #base_config = create_base_configuration()
    
    # Update with dynamic scenario
    config = update_with_dynamic_scenario(base_config, scenario, planning_problem)
    
    # Initialize logger and print configuration summary
    logger = util_logger.initialize_logger(config)
    config.print_configuration_summary()
    
    # Compute reachable sets
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()
    
    # Optionally visualize results
    # util_visual.plot_scenario_with_reachable_sets(reach_interface)
    
    return reach_interface