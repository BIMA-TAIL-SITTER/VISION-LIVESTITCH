import os
import unittest
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from src.utilities import extract_gps_data, extract_gps_data_tuple, extract_gps_data_dict, importData, display


class TestExtractGPS(unittest.TestCase):
    def setUp(self):
        self.test_images_dir = 'dataset/test_orthomosaics_42imgs/'
        self.valid_image = os.path.join(self.test_images_dir, 'G0735851.JPG')  # Replace with an actual image name
        
    def test_extract_gps_data_tuple(self):
        gps_data = extract_gps_data_tuple(self.valid_image)
        self.assertIsInstance(gps_data, tuple)
        self.assertEqual(len(gps_data), 3)  # Expecting (latitude, longitude, altitude)
        self.assertTrue(all(isinstance(coord, float) for coord in gps_data))

if __name__ == '__main__':
    unittest.main()