'''
Driver script. Execute this to perform the mosaic procedure.
'''
import argparse
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    
import utilities as util
from Combiner import Combiner
import cv2
import numpy as np

def main():
    parser = argparse.ArgumentParser(description='Create orthomosaic from images with GPS metadata')
    parser.add_argument('--image-dir', '-i', 
                        type=str, 
                        required=True,
                        help='Directory containing input images with GPS metadata')
    parser.add_argument('--output-dir', '-o', 
                        type=str, 
                        default='output',
                        help='Directory to save output results')
    parser.add_argument('--output-name', '-n', 
                        type=str, 
                        default='finalResult.png',
                        help='Name of the final output file')
    parser.add_argument('--method', '-m',
                        type=str,
                        choices=['gps', 'sequential'],
                        default='gps',
                        help='Stitching method: gps (recommended) or sequential')
    parser.add_argument('--gsd', '-g',
                        type=float,
                        default=0.1,
                        help='Ground Sample Distance in meters/pixel (default: 0.1m for ~100m altitude)')
    parser.add_argument('--no-blending', 
                        action='store_true',
                        help='Disable blending (only for sequential method)')
    
    args = parser.parse_args()
    
    # Ensure output directory exists
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"Loading images from {args.image_dir}...")
    allImages, gps_data = util.importData(args.image_dir, return_as_dict=True)
    
    if not allImages:
        print(f"❌ Error: No images found in {args.image_dir}")
        return
    
    print(f"✓ Loaded {len(allImages)} images")

    # Convert GPS to dataMatrix format
    dataMatrix = np.zeros((len(gps_data), 6))
    
    # Get origin from first image
    origin_lat = gps_data[0]["latitude"]
    origin_lon = gps_data[0]["longitude"]
    origin_alt = gps_data[0]["altitude"]
    
    print(f"Origin: Lat={origin_lat:.6f}, Lon={origin_lon:.6f}, Alt={origin_alt:.1f}m")
    
    for i, gps in enumerate(gps_data):
        # Convert lat/lon to local X/Y in meters
        # Longitude -> X (East), Latitude -> Y (North)
        x = (gps["longitude"] - origin_lon) * 111320 * np.cos(np.radians(origin_lat))
        y = (gps["latitude"] - origin_lat) * 110540
        z = gps["altitude"] - origin_alt

        dataMatrix[i, 0] = x  # X (East) in meters
        dataMatrix[i, 1] = y  # Y (North) in meters
        dataMatrix[i, 2] = z  # Z (altitude relative) in meters
        
        # Assume nadir (straight down) orientation
        dataMatrix[i, 3] = 0  # Yaw
        dataMatrix[i, 4] = 0  # Pitch
        dataMatrix[i, 5] = 0  # Roll

    # Create combiner
    print(f"\nCreating orthomosaic using {args.method.upper()} method...")
    myCombiner = Combiner(
        allImages, 
        dataMatrix, 
        args.output_dir, 
        use_blending=(not args.no_blending)
    )
    
    # Run appropriate stitching method
    if args.method == 'gps':
        result = myCombiner.createMosaicGPS(gsd=args.gsd)
    else:
        result = myCombiner.createMosaic()
    
    # Save final result
    if result is not None:
        output_path = os.path.join(args.output_dir, args.output_name)
        cv2.imwrite(output_path, result)
        print(f"\n✓ Orthomosaic saved to {output_path}")
        print(f"  Size: {result.shape[1]}x{result.shape[0]} pixels")
        
        # Display (comment out if running headless)
        # util.display("Final Orthomosaic", result)
    else:
        print("❌ Error: Orthomosaic generation failed.")
        
if __name__ == "__main__":
    main()