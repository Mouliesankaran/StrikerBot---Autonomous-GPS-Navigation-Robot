"""
GPS utility functions for navigation
"""

import math
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPose

def euler_from_quaternion(quaternion):
    """Convert a quaternion to euler angles"""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

def latLonYaw2Geopose(lat: float, lon: float, yaw: float = 0.0) -> GeoPose:
    """
    Convert latitude, longitude, and yaw to a GeoPose message
    """
    geopose = GeoPose()
    geopose.position.latitude = lat
    geopose.position.longitude = lon
    geopose.position.altitude = 0.0
    
    # Convert yaw to quaternion
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    geopose.orientation.x = qx
    geopose.orientation.y = qy
    geopose.orientation.z = qz
    geopose.orientation.w = qw
    
    return geopose

def euler_from_quaternion_msg(quaternion: Quaternion):
    """
    Convert a quaternion message to euler angles
    """
    return euler_from_quaternion(quaternion)

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS points in meters
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  # Earth radius in meters
    return c * r