import unittest
import os
import sys
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from src.utilities import extract_gps_data, extract_gps_data_tuple, extract_gps_data_dict, importData, display

class TestImportData(unittest.TestCase):
    def setUp(self):
        self.test_images_dir = 'dataset/test_orthomosaics_42imgs/'

    def test_import_data_as_dicts(self):
        allImages, dataMatrix = importData(self.test_images_dir, return_as_dict=True)

        # verify the number of images loaded
        self.assertEqual(len(allImages), 42)  # Assuming there are 42 images
        self.assertEqual(len(allImages), len(dataMatrix))
        
        # verify the metadata structure (dictionary)
        for metadata in dataMatrix:
            self.assertIsInstance(metadata, dict, "Metadata should be a dictionary!")
            self.assertIn("latitude", metadata, "Metadata should contain 'latitude' key!")
            self.assertIn("longitude", metadata, "Metadata should contain 'longitude' key!")
            self.assertIn("altitude", metadata, "Metadata should contain 'altitude' key!")
            self.assertIsInstance(metadata["latitude"], float, "'latitude' should be a float!")
            self.assertIsInstance(metadata["longitude"], float, "'longitude' should be a float!")
            self.assertIsInstance(metadata["altitude"], float, "'altitude' should be a float!")

if __name__ == '__main__':
    unittest.main()