'''
Driver script. Execute this to perform the mosaic procedure.
'''
import argparse
import os
import sys
# from memory_profiler import profile


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    
import utilities as util
import Combiner
import cv2
import numpy as np

# fileName = "dataset/imageData.txt"
# imageDirectory = "dataset/test_orthomosaics_42imgs"
# allImages, dataMatrix = util.importData(fileName, imageDirectory)
# myCombiner = Combiner.Combiner(allImages, dataMatrix)
# result = myCombiner.createMosaic()
# util.display("RESULT", result)
# cv2.imwrite("results/finalResult.png", result)

def main():
    # set up argument parser
    parser = argparse.ArgumentParser(description='Create orthomosaic from images with GPS metadata')
    parser.add_argument('--image-dir', '-i', 
                        type=str, 
                        default='dataset/test_orthomosaics_42imgs',
                        help='Directory containing input images with GPS metadata')
    parser.add_argument('--output-dir', '-o', 
                        type=str, 
                        default='output',
                        help='Directory to save output results')
    parser.add_argument('--output-name', '-n', 
                        type=str, 
                        default='finalResult.png',
                        help='Name of the final output file')
    
    args = parser.parse_args()
    
     # Ensure output directory exists
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"Loading images from {args.image_dir}...")
    allImages, gps_data = util.importData(args.image_dir, return_as_dict=True)
    
    if not allImages:
        print(f"Error: No images found in {args.image_dir}")
        return
    
    print(f"Loaded {len(allImages)} images")

    dataMatrix = np.zeros((len(gps_data), 6)) # Assuming 6 metadata fields for each image
    for i, gps in enumerate(gps_data):
        if i == 0:
            origin_lat , origin_lon, origin_alt = gps["latitude"], gps["longitude"], gps["altitude"]
            
        # convert lat/lon/alt to x,y,z
        x = (gps["longitude"] - origin_lon) * 111320 * np.cos(np.radians(origin_lat))  # meters
        y = (gps["latitude"] - origin_lat) * 110540  # meters
        z = gps["altitude"]  # altitude in meters

        # Set position values
        dataMatrix[i, 0] = x  # X (East)
        dataMatrix[i, 1] = y  # Y (North)
        dataMatrix[i, 2] = z  # Z (Altitude)
        
        # Assume nadir orientation (camera pointing straight down)
        dataMatrix[i, 3] = 0  # Yaw (degrees)
        dataMatrix[i, 4] = 0  # Pitch (degrees)
        dataMatrix[i, 5] = 0  # Roll (degrees)

    # create the orthomosaic
    print("Starting orthomosaic generation...")
    myCombiner = Combiner.Combiner(allImages, dataMatrix, args.output_dir)  # Pass output_dir
    result = myCombiner.createMosaic()
    
    if result is not None:
        util.display("RESULT", result)
        
        output_path = os.path.join(args.output_dir, args.output_name)
        cv2.imwrite(output_path, result)
        print(f"Orthomosaic saved to {output_path}")
    else:
        print("Error: Orthomosaic generation failed.")
        
if __name__ == "__main__":
    main()