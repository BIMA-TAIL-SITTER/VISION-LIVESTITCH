import cv2
import numpy as np
from PIL import Image
import os
import sys
import exifread
import logging
import json
import datetime
from pathlib import Path

# configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def extract_gps_data(image_path):
    """
    Extracts GPS data from image EXIF metadata with enhanced error handling.
    :param image_path: Path to the image file
    :return: A dictionary containing the GPS data or None if not found
    """
    try:
        with open(image_path, 'rb') as image_file:
            tags = exifread.process_file(image_file, details=False)

            # Try to extract basic GPS information
            if 'GPS GPSLatitude' in tags and 'GPS GPSLongitude' in tags:
                # Extract latitude and longitude
                lat = tags['GPS GPSLatitude'].values
                lon = tags['GPS GPSLongitude'].values

                try:
                    lat_ref = tags['GPS GPSLatitudeRef'].values
                except (KeyError, AttributeError):
                    logger.warning(f"No latitude reference in {image_path}, assuming North")
                    lat_ref = 'N'

                try:
                    lon_ref = tags['GPS GPSLongitudeRef'].values
                    if isinstance(lon_ref, list):
                        lon_ref = lon_ref[0]
                except (KeyError, AttributeError):
                    logger.warning(f"No longitude reference in {image_path}, assuming East")
                    lon_ref = 'E'

                # Convert to decimal degrees
                try:
                    latitude = convert_dms_to_decimal(lat, lat_ref)
                    longitude = convert_dms_to_decimal(lon, lon_ref)
                except Exception as e:
                    logger.error(f"Error converting GPS coordinates in {image_path}: {e}")
                    return None

                # Get altitude if available
                try:
                    if 'GPS GPSAltitude' in tags:
                        alt_value = tags['GPS GPSAltitude'].values[0]
                        altitude = float(alt_value.num) / float(alt_value.den)

                        # Check if we need to negate altitude
                        if 'GPS GPSAltitudeRef' in tags and tags['GPS GPSAltitudeRef'].values in [1, '1']:
                            altitude = -altitude
                    else:
                        altitude = 0.0
                except Exception as e:
                    logger.warning(f"Could not parse altitude in {image_path}: {e}")
                    altitude = 0.0

                # Extract timestamp if available
                try:
                    if 'GPS GPSTimeStamp' in tags and 'GPS GPSDateStamp' in tags:
                        time_value = tags['GPS GPSTimeStamp'].values
                        date_value = str(tags['GPS GPSDateStamp'].values)

                        hours = float(time_value[0].num) / float(time_value[0].den)
                        minutes = float(time_value[1].num) / float(time_value[1].den)
                        seconds = float(time_value[2].num) / float(time_value[2].den)

                        timestamp = f"{date_value} {int(hours):02d}:{int(minutes):02d}:{seconds:.2f}"
                    else:
                        timestamp = None
                except Exception as e:
                    logger.warning(f"Could not parse timestamp in {image_path}: {e}")
                    timestamp = None

                # Check if camera orientation data is available
                orientation_data = extract_orientation_data(tags)

                result = {
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude": altitude
                }

                if timestamp:
                    result["timestamp"] = timestamp

                if orientation_data:
                    result.update(orientation_data)

                return result
            else:
                logger.warning(f"No GPS data found in {image_path}")
                return None

    except Exception as e:
        logger.error(f"Error processing {image_path}: {e}")
        return None
    

def convert_dms_to_decimal(dms_value, ref):
    """
    Convert degrees, minutes, seconds to decimal degrees.
    :param dms: Tuple of (degrees, minutes, seconds)
    :param ref: 'N', 'S', 'E', or 'W'
    :return: Decimal degrees
    """
    try:
        # Handle case where we have rational values
        if hasattr(dms_value[0], 'num'):
            degrees = float(dms_value[0].num) / float(dms_value[0].den)
            minutes = float(dms_value[1].num) / float(dms_value[1].den) if len(dms_value) > 1 else 0
            seconds = float(dms_value[2].num) / float(dms_value[2].den) if len(dms_value) > 2 else 0
        # Handle case where we have direct values
        else:
            degrees = float(dms_value[0])
            minutes = float(dms_value[1]) if len(dms_value) > 1 else 0
            seconds = float(dms_value[2]) if len(dms_value) > 2 else 0

        decimal = degrees + (minutes / 60.0) + (seconds / 3600.0)

        # Apply sign based on reference
        if ref in ['S', 'W', 's', 'w']:
            decimal = -decimal

        return decimal
    except Exception as e:
        raise ValueError(f"Could not convert DMS to decimal: {e}")

def extract_orientation_data(tags):
    """
    Extract camera orientation data from EXIF tags if available.
    :param tags: EXIF tags
    :return: Dictionary with orientation data or default values
    """
    orientation = {}

    # Try to find orientation data in different possible tag locations
    for yaw_tag in ['Exif.Custom.Yaw', 'XMP:FlightYawDegree', 'Drone:FlightYawDegree']:
        if yaw_tag in tags:
            try:
                orientation['yaw'] = float(str(tags[yaw_tag].values))
                break
            except (ValueError, AttributeError):
                pass

    for pitch_tag in ['Exif.Custom.Pitch', 'XMP:GimbalPitchDegree', 'Drone:GimbalPitchDegree']:
        if pitch_tag in tags:
            try:
                orientation['pitch'] = float(str(tags[pitch_tag].values))
                break
            except (ValueError, AttributeError):
                pass

    for roll_tag in ['Exif.Custom.Roll', 'XMP:GimbalRollDegree', 'Drone:GimbalRollDegree']:
        if roll_tag in tags:
            try:
                orientation['roll'] = float(str(tags[roll_tag].values))
                break
            except (ValueError, AttributeError):
                pass

    # If no orientation data is found, return default values
    if not orientation:
        orientation = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}

    return orientation


def importData(imageDirectory, return_as_dict=True, sort_by_name=True):
    """
    Import image data and metadata directly from image files with improved error handling.
    :param imageDirectory: Directory where images are stored.
    :param return_as_dict: If True, return metadata as dictionaries; otherwise, return as tuples.
    :param sort_by_name: If True, sort images by filename
    :return: allImages: List of images as NumPy arrays.
             dataMatrix: List of metadata (tuples or dictionaries).
    """
    allImages = []
    metadata_list = []
    valid_extensions = ('.png', '.jpg', '.jpeg', '.tiff', '.tif', '.bmp')

    # Get list of image files
    file_names = [f for f in os.listdir(imageDirectory) if f.lower().endswith(valid_extensions)]

    if sort_by_name:
        file_names.sort()

    # Process each file
    for file_name in file_names:
        image_path = os.path.join(imageDirectory, file_name)
        try:
            # Read the image
            image = cv2.imread(image_path)
            if image is None:
                logger.warning(f"Could not read image: {image_path}")
                continue

            allImages.append(image)

            # Extract metadata
            metadata = extract_gps_data(image_path)

            if metadata:
                metadata_list.append(metadata)
            else:
                logger.warning(f"No metadata found for {file_name}. Using default values.")
                if return_as_dict:
                    metadata_list.append({'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0})
                else:
                    metadata_list.append((0.0, 0.0, 0.0))  # Default values for tuple

        except Exception as e:
            logger.error(f"Error processing {image_path}: {e}")

    logger.info(f"Loaded {len(allImages)} images from {imageDirectory}")
    return allImages, metadata_list


def display(title, image, wait_time=0):
    '''
    OpenCV machinery for showing an image.
    :param title: Window title in string form
    :param image: ndArray containing image to show
    :param wait_time: Time to wait in milliseconds (0 = wait for key press)
    '''
    # Check image dimensions and resize if too large
    h, w = image.shape[:2]
    max_dim = 1920  # Maximum window dimension

    if h > max_dim or w > max_dim:
        scale = min(max_dim / h, max_dim / w)
        new_h, new_w = int(h * scale), int(w * scale)
    else:
        new_h, new_w = h, w

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, new_w, new_h)
    cv2.imshow(title, image)
    key = cv2.waitKey(wait_time)

    # Return pressed key in case caller needs it
    return key

def drawMatches(img1, kp1, img2, kp2, matches, max_matches=100):
    """
    Makes an image with matched features denoted.
    :param img1: First image
    :param kp1: Keypoints from first image
    :param img2: Second image
    :param kp2: Keypoints from second image
    :param matches: List of DMatch objects
    :param max_matches: Maximum number of matches to draw (to avoid cluttering)
    :return: Image with matches drawn
    """
    # Use OpenCV's built-in function if available (OpenCV 3+)
    try:
        # Only show the best matches
        if len(matches) > max_matches:
            matches = sorted(matches, key=lambda x: x.distance)[:max_matches]

        return cv2.drawMatches(img1, kp1, img2, kp2, matches, None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    except AttributeError:
        # Fallback for older OpenCV versions
        # Create a new output image
        rows1, cols1 = img1.shape[:2]
        rows2, cols2 = img2.shape[:2]

        out = np.zeros((max(rows1, rows2), cols1 + cols2, 3), dtype='uint8')

        # Place the first image to the left
        if len(img1.shape) == 2:  # Grayscale
            out[:rows1, :cols1] = np.dstack([img1, img1, img1])
        else:  # Color
            out[:rows1, :cols1] = img1

        # Place the second image to the right
        if len(img2.shape) == 2:  # Grayscale
            out[:rows2, cols1:] = np.dstack([img2, img2, img2])
        else:  # Color
            out[:rows2, cols1:] = img2

        # Limit number of matches to draw
        if len(matches) > max_matches:
            matches = sorted(matches, key=lambda x: x.distance)[:max_matches]

        # Draw matches
        for m in matches:
            # Get keypoint coordinates
            if hasattr(m, 'queryIdx'):
                img1_idx = m.queryIdx
                img2_idx = m.trainIdx
            else:
                img1_idx = m[0].queryIdx
                img2_idx = m[0].trainIdx

            (x1, y1) = kp1[img1_idx].pt
            (x2, y2) = kp2[img2_idx].pt

            # Draw circles and line
            radius = 4
            thickness = 1
            color = (0, 255, 0)  # Green
            cv2.circle(out, (int(x1), int(y1)), radius, color, thickness)
            cv2.circle(out, (int(x2) + cols1, int(y2)), radius, color, thickness)
            cv2.line(out, (int(x1), int(y1)), (int(x2) + cols1, int(y2)), color, thickness)

        return out

def save_metadata(metadata_list, output_file):
    """
    Save metadata to a JSON file for later reference
    :param metadata_list: List of metadata dictionaries
    :param output_file: Path to output JSON file
    """
    try:
        with open(output_file, 'w') as f:
            json.dump({
                'timestamp': datetime.datetime.now().isoformat(),
                'count': len(metadata_list),
                'metadata': metadata_list
            }, f, indent=2)
        logger.info(f"Metadata saved to {output_file}")
    except Exception as e:
        logger.error(f"Error saving metadata: {e}")