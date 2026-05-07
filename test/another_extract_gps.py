import os
from src.utilities import extract_gps_data_tuple, extract_gps_data_dict

def test_extract_gps(imageDirectory):
    """
    Test the extract_gps_data_tuple and extract_gps_data_dict functions on a dataset.
    :param imageDirectory: Path to the directory containing the images.
    """
    print(f"Testing GPS extraction on images in: {imageDirectory}\n")

    for file_name in sorted(os.listdir(imageDirectory)):
        image_path = os.path.join(imageDirectory, file_name)
        if file_name.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp', '.gif')):
            print(f"Processing: {file_name}")

            # Test extract_gps_data_tuple
            gps_tuple = extract_gps_data_tuple(image_path)
            if gps_tuple:
                print(f"  GPS Data (Tuple): {gps_tuple}")
            else:
                print("  GPS Data (Tuple): No metadata found.")

            # Test extract_gps_data_dict
            gps_dict = extract_gps_data_dict(image_path)
            if gps_dict:
                print(f"  GPS Data (Dictionary): {gps_dict}")
            else:
                print("  GPS Data (Dictionary): No metadata found.")

if __name__ == "__main__":
    # Path to your dataset
    imageDirectory = "dataset/test_orthomosaics_42imgs"
    test_extract_gps(imageDirectory)