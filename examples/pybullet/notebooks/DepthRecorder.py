import cv2
#import cv_bridge
import numpy as np
import rospy
import snappy
import struct

class DepthRecorder(object):
    def __init__(self):
        """ Depth recorder class writing depth ROS image messages (single
        channel Float32 image) recorded with, e.g., the Kinect V2 sensor,
        compressed into a .bin binary file and time stamps for each image
        frame into an accompanying .txt file.
        """
        #self._topic = '/cameras/kinect/depth/image_raw'
        #self._msg_type = Image
        self._fp_d = None
        self._fp_ts = None
        #self._sub = None
        self._running = False

    def clean_shutdown(self):
        """ Clean shutdown of the depth recorder. """
        self._close_binary()
        self._close_file()
        self._running = False

    def _close_binary(self):
        """ Close binary file we wrote depth image frames into. """
        if self._fp_d:
            self._fp_d.close()
            self._fp_d = None


    def _close_file(self):
        """ Close text file we wrote time stamps into. """
        if self._fp_ts:
            self._fp_ts.close()
            self._fp_ts = None


    def start(self, outname):
        """ Set up the depth recorder with the parameters for the recording
         of the depth sensor.
        :param outname: Filename to write the binary and text files to,
        without the extension.
        :return: Whether the binary- and text file were opened successfully.
        """
        if not self._running:
                # open text file
                try:
                    self._fp_ts = open(outname + '.txt', 'w')
                except IOError as e:
                     raise e
                self._fp_ts.write('# timestamps [s]\n')
                # open binary file
                try:
                    self._fp_d = open(outname + '.bin', 'wb')
                except IOError as e:
                     raise e

        self._running = True

        return not (self._fp_d.closed and self._fp_ts.closed)



    def stop(self):
        """ Stop recording data from the depth sensor.
        :return: Whether the binary- and text file are open.
        """
        if self._running:
                self._close_binary()
                self._close_file()
                self._running = False

        return not (self._fp_d.closed and self._fp_ts.closed)



    def _add_image(self, ts, img):
        """ Add depth image to archive """

        # write time stamp to file
        self._fp_ts.write('%f\n' % ts)
        self._fp_ts.flush()
        # add frame to binary file
        #try:
        #    img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg)
        #except cv_bridge.CvBridgeError as e:
        #    rospy.logfatal("'%s' Failed to convert ROS image message!" % self)
        #    raise e
        if img.dtype == np.float32:
            # Is simulated data
            mask = np.isnan(img)
            if mask.any():
                # In simulation, the background has NaN depth values.
                # We replace them with 0 m, similar to what the Kinect V1 did.
                # See https://msdn.microsoft.com/en-us/library/jj131028.aspx.
                #rospy.logdebug("There was at least one NaN in the depth image. " +
                #              "I replaced all occurrences with 0.0 m.")
                img.flags.writeable = True
                img[mask] = 0.0
                # We now map the float values in meters to uint16 values in mm
                # as provided by the libfreenect2 library and Kinect SDK.
                img *= 1000.0
                img = img.astype(np.uint16, copy=False)
        assert img.dtype == np.uint16

        # compress image with snappy
        img_comp = snappy.compress(img)
        # write number of bytes of compressed image
        nr_bytes = struct.pack('<L', len(img_comp))
        self._fp_d.write(nr_bytes)
        # write compressed image
        self._fp_d.write(img_comp)
        self._fp_d.flush()

    def depth_from_binary(binary_name, imgsize=(240, 320)):
        """ Decode binary file containing depth images and return the depth
        images as a numpy ndarray.
        :param binary_name: The file name of the binary file to read.
        :param imgsize: The size (height, width) of each uncompressed image.
        :return: numpy array containing 'l' images of size 'imgsize'.
        """
        images = list()
        with open(binary_name, 'rb') as fp:
            b = fp.read(4)
            while b != '':
                k = struct.unpack('<L', b)[0]
                image_bytes = fp.read(k)
                images.append(snappy.uncompress(image_bytes))
                b = fp.read(4)
        l = len(images)
        images = np.array(images)
        images = np.fromstring(images, dtype=np.dtype('>u2'))
        return images.reshape((l,) + imgsize)



#for testing of the compress/decompress functionality
if __name__ == '__main__':
    import cv2

    # fn = '/home/baxter/Downloads/DepthSenseDepthLog2015-12-17 13.13.19.647.bin'
    fn = '/home/baxter/ros_ws/src/baxter_data_acquisition/data/201603221452-0_kinect_depth_depth.bin'

    imgs = depth_from_binary(fn, (424, 512))

    for i in range(imgs.shape[0]):
        cv2.imshow('depthimage', imgs[i, :, :])
        cv2.waitKey(0)
    cv2.destroyAllWindows()