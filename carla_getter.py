from carla import Actor
import carla
import math



def carla_to_commonroad_transform_actor(ego_vehicle: Actor) -> tuple:
    transform = ego_vehicle.get_transform()
    
    # Position conversion (X remains same, Y inverted)
    position = [
        transform.location.x, 
        -transform.location.y  # Flip Y-axis
    ]
    
    # Orientation conversion
    orientation = get_commonroad_2d_orientation(ego_vehicle)
    
    return position, orientation

def get_commonroad_2d_orientation(ego_vehicle: Actor) -> float:
    """Converts CARLA vehicle rotation to CommonRoad 2D orientation (radians)"""
    # Get CARLA transform with yaw in degrees (left-handed system)
    carla_transform = ego_vehicle.get_transform()
    
    # Convert to CommonRoad right-handed system
    # 1. Negate yaw to account for coordinate system flip (left → right-handed)
    # 2. Convert degrees to radians
    commonroad_yaw = -math.radians(carla_transform.rotation.yaw)
    
    # Normalize to [-π, π]
    return (commonroad_yaw + math.pi) % (2 * math.pi) - math.pi




def carla_to_commonroad_transform_actor_manual(ego_vehicle) -> tuple:

    # Position conversion (X remains same, Y inverted)
    position = [
        ego_vehicle[0][0], 
        -ego_vehicle[0][1]  # Flip Y-axis
    ]
    
    # Orientation conversion
    orientation = get_commonroad_2d_orientation_manual(ego_vehicle)
    
    return position, orientation

def get_commonroad_2d_orientation_manual(ego_vehicle) -> float:
    """Converts CARLA vehicle rotation to CommonRoad 2D orientation (radians)"""
    
    # Convert to CommonRoad right-handed system
    # 1. Negate yaw to account for coordinate system flip (left → right-handed)
    # 2. Convert degrees to radians
    commonroad_yaw = -math.radians(ego_vehicle[1])
    
    # Normalize to [-π, π]
    return (commonroad_yaw + math.pi) % (2 * math.pi) - math.pi