import os
import platform
import numpy as np
import cv2
import asyncio

# Use of Harvester to access the camera.
# For more information regarding Harvester, visit the github page:
# https://github.com/genicam/harvesters
from harvesters.core import Harvester

BAYER_FORMATS = {"BayerGR8": cv2.COLOR_BayerGR2RGB,
                 "BayerRG8": cv2.COLOR_BayerRG2RGB,
                 "BayerBG8": cv2.COLOR_BayerBG2RGB,
                 "BayerGB8": cv2.COLOR_BayerGB2RGB}

def find_producer(name):
    """ Helper for the GenTL producers from the environment path.
    """
    # paths = []
    # paths.append('/opt/pylon/lib/gentlproducer/gtl/')

    paths = os.environ['GENICAM_GENTL64_PATH'].split(os.pathsep)

    if platform.system() == "Linux":
        paths.append('/opt/pylon/lib/gentlproducer/gtl/')

    for path in paths:
        path += os.path.sep + name
        if os.path.exists(path):
            return path
    return ""

class CapturingModule:
    def __init__(self):
        # Create Harvester instances.
        self.h = Harvester()

        # Location of the Basler blaze GenTL producer.
        if platform.system() == "Windows" or platform.system() == "Linux":
            path_to_gev_cti = find_producer("ProducerGEV.cti")
        else:
            print(f"{platform.system()} is not supported")
            assert False

        # Add producer to Harvester.
        assert os.path.exists(path_to_gev_cti)

        self.h.add_file(path_to_gev_cti)

        # Update device list.
        self.h.update()

    def setup_2Dcamera(self):
        # Connect to the first available 2D camera. Ignore blaze cameras, which will
        # be enumerated as well.
        dev_info = next(
            (d for d in self.h.device_info_list if 'blaze' not in d.model), None)
        
        print("dev_info: " , dev_info)
        if dev_info is not None:
            self.ia_gev = self.h.create(
                {"serial_number": dev_info.serial_number})
            # print("Connected to ace camera: {}".format(
            #     dev_info.serial_number))
        else:
            print("No 2D camera found.")
            raise RuntimeError

        # Figure out which 8-bit Bayer pixel format the camera supports.
        # If the camera supports an 8-bit Bayer format, enable the format.
        # Otherwise, exit the program.
        bayer_pattern = next((pf for pf in self.ia_gev.remote_device.node_map.PixelFormat.symbolics
                              if pf in BAYER_FORMATS), None)
        if bayer_pattern is not None:
            self.ia_gev.remote_device.node_map.PixelFormat.value = bayer_pattern
        else:
            print("The camera does not provide Bayer pattern-encoded 8-bit color images.")
            raise RuntimeError

        # Configure the camera for software triggering.
        # Each software trigger will start the acquisition of one single frame.

        self.ia_gev.remote_device.node_map.AcquisitionMode.value = "SingleFrame"
        self.ia_gev.remote_device.node_map.TriggerSelector.value = "FrameStart"
        self.ia_gev.remote_device.node_map.TriggerMode.value = "On"
        self.ia_gev.remote_device.node_map.TriggerSource.value = "Software"
        self.ia_gev.remote_device.node_map.BalanceWhiteAuto.value = "Once"
        self.ia_gev.remote_device.node_map.GainRaw.value = 0
        self.ia_gev.remote_device.node_map.ExposureTimeAbs.value = 10000

    def close_2DCamera(self):
        # Stop image acquisition.
        self.ia_gev.stop()
        self.ia_gev.remote_device.node_map.TriggerMode.value = "Off"

        # Disconnect from camera.
        self.ia_gev.destroy()

    def close_harvesters(self):
        # Remove the CTI file and reset Harvester.
        self.h.reset()

    def get_image_2DCamera(self):
        self.ia_gev.remote_device.node_map.TriggerMode.value = "On"
        self.ia_gev.remote_device.node_map.TriggerSource.value = "Software"
        self.ia_gev.start()
        with self.ia_gev.fetch(timeout=0.05) as buffer:
            # Warning: The buffer is only valid in the with statement and will be destroyed
            # when you leave the scope.
            # If you want to use the buffers outside of the with scope, you have to use np.copy()
            # to make a deep copy of the image.

            # Create an alias of the image component:
            image = buffer.payload.components[0]

            # Reshape the image into a 2D array:
            _2d = image.data.reshape(image.height, image.width)

            # Debayer the image to a grayscale image.
            gray = cv2.cvtColor(_2d, BAYER_FORMATS[image.data_format])

            return gray.copy()

    def run(self, return_intensity=False):
        self.ia_gev.remote_device.node_map.TriggerSoftware.execute()

        try:
            color_img = self.get_image_2DCamera()
            return color_img
        except Exception as e:
            # traceback.print_exc()
            return None

    def setup(self):
        # Set up the cameras.
        self.setup_2Dcamera()

    def close(self):
        # Close the camera and release the producers.
        self.close_2DCamera()
        self.close_harvesters()
