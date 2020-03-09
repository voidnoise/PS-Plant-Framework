import PyCapture2
import cv2
import numpy as np


def print_build_info():
    lib_ver = PyCapture2.getLibraryVersion()
    print('PyCapture2 library version: %d %d %d %d' % (lib_ver[0], lib_ver[1], lib_ver[2], lib_ver[3]))
    print()


def print_camera_info(cam):
    cam_info = cam.getCameraInfo()
    print('\n*** CAMERA INFORMATION ***\n')
    print('Serial number - %d' % cam_info.serialNumber)
    print('Camera model - %s' % cam_info.modelName)
    print('Camera vendor - %s' % cam_info.vendorName)
    print('Sensor - %s' % cam_info.sensorInfo)
    print('Resolution - %s' % cam_info.sensorResolution)
    print('Firmware version - %s' % cam_info.firmwareVersion)
    print('Firmware build time - %s' % cam_info.firmwareBuildTime)
    print()



class FlirGrasshopper():

    def __init__(self, cams=[0,1]):
        # Print PyCapture2 Library Information
        print_build_info()
        self.bus = PyCapture2.BusManager()
        num_cams = self.bus.getNumOfCameras()
        print('Number of cameras detected: ', num_cams)
        if not num_cams:
            print('Insufficient number of cameras. Exiting...')
            exit()

        # Select camera on 0th index
        self.cameras = [PyCapture2.Camera() for i in range(len(cams))]
        for i,camera in enumerate(self.cameras):
            camera.connect(self.bus.getCameraFromIndex(cams[i]))
            print_camera_info(camera)

        self.start_capture()          
    
    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.stop_capture()
        self.disconnect()

    def start_capture(self):
        for i,camera in enumerate(self.cameras):
            camera.startCapture()

    def stop_capture(self):
        for i,camera in enumerate(self.cameras):
            camera.stopCapture()

    def disconnect(self):
        for i,camera in enumerate(self.cameras):
            camera.disconnect()

    def grab_image(self,camera=0,scale=None):
        cam = self.cameras[camera]
        try:
            image = cam.retrieveBuffer()
        except PyCapture2.Fc2error as fc2Err:
            print('Error retrieving buffer : %s' % fc2Err)
            return None
        newimg = image.convert(PyCapture2.PIXEL_FORMAT.BGR)
        data = np.array(newimg.getData(),dtype='uint8')
        data = data.reshape( newimg.getRows(), newimg.getCols(), 3)
        if scale:
            w = int(data.shape[1]*scale)
            h = int(data.shape[0]*scale)
            data = cv2.resize(data,(h,w),interpolation = cv2.INTER_AREA)
        return data

    def grab_frames(self,scale=None):
        """ grab images from all cameras
        """
        return [self.grab_image(camera=i,scale=scale) for i in range(len(self.cameras))]

    def show_img(self,data,window='frame',scale=1.0):
        w = int(data.shape[1]*scale)
        h = int(data.shape[0]*scale)
        img = cv2.resize(data,(h,w),interpolation = cv2.INTER_AREA)
        cv2.imshow('frame',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
	fg = FlirGrasshopper()