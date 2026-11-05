"""
GPS utilities for parsing EXIF data and calculating distances.
"""
import io
import math
import logging
from typing import Union, Tuple, Dict, Any
from pathlib import Path
import exifread


logger = logging.getLogger(__name__)


def _convert_to_degrees(value: Any) -> float:
    """
    Helper function to convert the GPS coordinates stored in the EXIF to degress in float format
    """
    try:
        d = float(value.values[0].num) / float(value.values[0].den)
        m = float(value.values[1].num) / float(value.values[1].den)
        s = float(value.values[2].num) / float(value.values[2].den)
        return d + (m / 60.0) + (s / 3600.0)
    except (IndexError, ZeroDivisionError, AttributeError, ValueError) as e:
        raise ValueError(f"Failed to parse GPS degrees: {e}")


def extract_gps_exif(source: Union[str, Path, bytes]) -> Dict[str, float]:
    """
    Parse GPS from EXIF.

    Args:
        source: File path (str/Path) or bytes.

    Returns:
        A dictionary with 'latitude', 'longitude', and 'altitude' in float.

    Raises:
        ValueError: If GPS data is missing or cannot be parsed.
    """
    try:
        if isinstance(source, bytes):
            f = io.BytesIO(source)
            tags = exifread.process_file(f, details=False)
        else:
            with open(source, 'rb') as f:
                tags = exifread.process_file(f, details=False)
    except Exception as e:
        raise ValueError(f"Failed to read EXIF data: {e}")

    if 'GPS GPSLatitude' not in tags or 'GPS GPSLongitude' not in tags:
        raise ValueError("GPS data missing from EXIF.")

    try:
        lat = _convert_to_degrees(tags['GPS GPSLatitude'])
        if 'GPS GPSLatitudeRef' in tags and tags['GPS GPSLatitudeRef'].values[0] != 'N':
            lat = -lat

        lon = _convert_to_degrees(tags['GPS GPSLongitude'])
        if 'GPS GPSLongitudeRef' in tags and tags['GPS GPSLongitudeRef'].values[0] != 'E':
            lon = -lon
            
        alt = 0.0
        if 'GPS GPSAltitude' in tags:
            alt_tag = tags['GPS GPSAltitude']
            alt = float(alt_tag.values[0].num) / float(alt_tag.values[0].den)
            if 'GPS GPSAltitudeRef' in tags:
                alt_ref = tags['GPS GPSAltitudeRef'].values[0]
                # EXIF standard: 0 = above sea level, 1 = below sea level.
                if isinstance(alt_ref, int) and alt_ref == 1:
                    alt = -alt
                elif isinstance(alt_ref, bytes) and alt_ref == b'\x01':
                    alt = -alt

        return {"latitude": lat, "longitude": lon, "altitude": alt}
    except Exception as e:
        raise ValueError(f"Error parsing GPS coordinates: {e}")


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two points on the Earth surface.

    Args:
        lat1: Latitude of first point in decimal degrees.
        lon1: Longitude of first point in decimal degrees.
        lat2: Latitude of second point in decimal degrees.
        lon2: Longitude of second point in decimal degrees.

    Returns:
        Distance in meters.
    """
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


def gps_to_local_meters(
    lat: float, lon: float, alt: float, 
    origin_lat: float, origin_lon: float, origin_alt: float
) -> Tuple[float, float, float]:
    """
    Convert WGS-84 to local ENU (East-North-Up) meters relative to an origin.

    Args:
        lat: Target latitude in decimal degrees.
        lon: Target longitude in decimal degrees.
        alt: Target altitude in meters.
        origin_lat: Origin latitude in decimal degrees.
        origin_lon: Origin longitude in decimal degrees.
        origin_alt: Origin altitude in meters.

    Returns:
        Tuple of (x_east_m, y_north_m, z_up_m).
    """
    origin_lat_rad = math.radians(origin_lat)
    x = (lon - origin_lon) * 111320.0 * math.cos(origin_lat_rad)
    y = (lat - origin_lat) * 110540.0
    z = alt - origin_alt
    return float(x), float(y), float(z)


def estimate_pixel_offset(
    lat: float, lon: float, 
    prev_lat: float, prev_lon: float, 
    gsd_m_per_px: float
) -> Tuple[float, float]:
    """
    Estimate pixel offset from GPS delta.

    Args:
        lat: Current latitude in decimal degrees.
        lon: Current longitude in decimal degrees.
        prev_lat: Previous latitude in decimal degrees.
        prev_lon: Previous longitude in decimal degrees.
        gsd_m_per_px: Ground Sampling Distance in meters per pixel.

    Returns:
        Tuple of (dx_pixels, dy_pixels).
    """
    if gsd_m_per_px <= 0:
        raise ValueError("GSD must be greater than 0.")
        
    dx_m, dy_m, _ = gps_to_local_meters(
        lat=lat, lon=lon, alt=0.0,
        origin_lat=prev_lat, origin_lon=prev_lon, origin_alt=0.0
    )
    
    return float(dx_m / gsd_m_per_px), float(dy_m / gsd_m_per_px)
