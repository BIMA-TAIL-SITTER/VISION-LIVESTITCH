import numpy as np
import cv2
import math as m
from scipy.spatial.transform import Rotation as Rot

def computeUnRotMatrix(pose):
    '''
    Compute a rotation matrix to correct for camera orientation.
    Enhanced to handle different coordinate systems and error checking.
    :param pose: A 1x6 NumPy ndArray containing pose information in [X,Y,Z,Yaw,Pitch,Roll] format
    :return: A 3x3 rotation matrix
    '''
    
    if pose is None or len(pose) < 6:
        raise ValueError("Pose must be a 1x6 array containing [X,Y,Z,Yaw,Pitch,Roll].")
    
    yaw = pose[3] * np.pi / 180 # alpha - rotation around Z axis
    pitch = pose[4] * np.pi / 180 # beta - rotation around Y axis
    roll = pose[5] * np.pi / 180 # gamma - rotation around X axis

    # option 1: Using scipy for rotation matrix computation
    try:
        rot = Rot.from_euler('zyx', [yaw, pitch, roll])
        rotation_matrix = rot.as_matrix()
        
        # For orthophoto generation, we only want to correct for roll and pitch
        # but keep yaw information as it indicates the heading 
        
        rotation_matrix[0, 2] = 0
        rotation_matrix[1, 2] = 0
        rotation_matrix[2, 2] = 1

        # return inverse matrix to undo the rotation
        return np.linalg.inv(rotation_matrix)
    except:
        #  option 2: Manual computation if scipy fails
        Rz = np.array([
            [m.cos(yaw), -m.sin(yaw), 0],
            [m.sin(yaw), m.cos(yaw), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [m.cos(pitch), 0, m.sin(pitch)],
            [0, 1, 0],
            [-m.sin(pitch), 0, m.cos(pitch)]
        ])

        Rx = np.array([
            [1, 0, 0],
            [0, m.cos(roll), -m.sin(roll)],
            [0, m.sin(roll), m.cos(roll)]
        ])

        # Combine rotations - order is important: first roll, then pitch, then yaw
        R = Rz @ Ry @ Rx

        # For orthophoto generation, we only want to correct for roll and pitch
        R[0, 2] = 0
        R[1, 2] = 0
        R[2, 2] = 1

        # Return inverse to undo the rotation
        return np.linalg.inv(R)
    
def warpPerspectiveWithPadding(image, transformation):
    '''
    Applies perspective transformation with padding to ensure the entire transformed image is visible.
    :param image: ndArray image
    :param transformation: 3x3 ndArray representing perspective transformation
    :return: transformed image
    '''
    
    # handle invalid inputs
    if image is None or transformation is None:
        return image
    
    # get image dimensions 
    height, width = image.shape[:2]
    
    # define the four corners of the image
    corners = np.float32([[0, 0], [0, height], [width, height], [width, 0]]).reshape(-1, 1, 2)
    
    try:
        warpedCorners = cv2.perspectiveTransform(corners, transformation)
    except Exception as e:
        print(f"Error occurred during perspective transformation: {e}")
        return image
    
    # Find min/max coordinates to determine new image size
    [xMin, yMin] = np.int32(warpedCorners.min(axis=0).ravel() - 0.5)
    [xMax, yMax] = np.int32(warpedCorners.max(axis=0).ravel() + 0.5)

    # Create translation matrix to move to positive coordinates
    translation = np.array([
        [1, 0, -xMin],
        [0, 1, -yMin],
        [0, 0, 1]
    ])

    # Combine transformations
    fullTransformation = translation @ transformation

    # Apply transformation
    result = cv2.warpPerspective(image, fullTransformation, (xMax - xMin, yMax - yMin))

    return result

def gps_to_local_coords(gps_points, reference_point=None):
    '''
    Convert GPS coordinates to local Cartesian coordinates.
    :param gps_points: List of (lat, lon, alt) tuples
    :param reference_point: Reference (lat, lon, alt) tuple. If None, uses the first point.
    :return: NumPy array of (x, y, z) coordinates in meters
    '''
    
    if not gps_points:
        return np.array([])
    
    # if no reference point is provided, use the first GPS point
    if reference_point is None:
        reference_point = gps_points[0]
        
    ref_lat, ref_lon, ref_alt = reference_point
    
    # earth radius in meters
    earth_radius  = 6378137.0
    
    # convert to local coordinates 
    coords = []
    for lat, lon, alt in gps_points:
        d_lat = np.radians(lat - ref_lat)
        d_lon = np.radians(lon - ref_lon)
        
        x = d_lon * earth_radius * np.cos(np.radians(ref_lat))
        y = d_lat * earth_radius
        z = alt - ref_alt
        
        coords.append((x, y, z))

    return np.array(coords)

