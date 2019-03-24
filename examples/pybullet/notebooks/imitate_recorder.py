import time
import os
import csv
import sys
import itertools
import numpy as np
import cv2
import snappy
import struct

class ImitationRecorder():
    def __init__(self, task):
        # Create the appropriate directory in the datas for the task we are training
        if not os.path.exists('D:\\Mybullet\\examples\\pybullet\\notebooks\\datas\\' + task + '\\'):
            os.mkdir('D:\\Mybullet\\examples\\pybullet\\notebooks\\datas\\' + task + '\\')
        self.save_folder = None
        self.save_rgb_folder = None  # The specific folder
        self.save_depth_folder = None  # The specific folder
        self.save_semantic_folder = None  # The specific folder
        self.writer = None  # The writer to create our txt files
        self.text_file = None  # The file that we are writing to currently
        self.is_recording = False  # Toggling recording
        self.task = task
        self.creation_time = time.time()
        self.fp_d = None

    def toggle_recording(self, toggle):
        if toggle is "0":
            self.is_recording = False

            if self.text_file != None:
                self.text_file.close()
                self.text_file = None

            if self.fp_d != None:
                self.fp_d.close()
                self.fp_d = None
            print("-----Stop Recording-----")
        else:
            #ft = time.time()
            save_folder = 'D:\\Mybullet\\examples\\pybullet\\notebooks\\datas\\' + self.task + '\\' + str(self.creation_time) + '\\'
            if not os.path.exists(save_folder):
                os.mkdir(save_folder)
            self.save_folder = save_folder

            save_rgb_folder = save_folder+'rgb'+'\\'
            if not os.path.exists(save_rgb_folder):
                os.mkdir(save_rgb_folder)
            self.save_rgb_folder = save_rgb_folder

            save_depth_folder = save_folder + 'depth' + '\\'
            if not os.path.exists(save_depth_folder):
                os.mkdir(save_depth_folder)
            self.save_depth_folder = save_depth_folder

            save_semantic_folder = save_folder + 'semantic' + '\\'
            if not os.path.exists(save_semantic_folder):
                os.mkdir(save_semantic_folder)
            self.save_semantic_folder = save_semantic_folder

            self.text_file = open(save_folder + 'vectors.txt', 'w')
            self.writer = csv.writer(self.text_file)
            try:
                self.fp_d = open(save_depth_folder + 'depth.bin', 'wb')
            except IOError as e:
                raise e
            self.is_recording = True
            print("=====Start Recording=====")

    def record_video_data(self, step, rgb_image, depth_image, semantic_image=None):
        cv2.imwrite(self.save_rgb_folder + str(step) + '_rgb.png', rgb_image)
        print("stored image for step "+str(step))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imwrite(self.save_depth_folder + str(step) + '_depth.png', depth_colormap)

        depth_img_comp = snappy.compress(depth_image)
        # write number of bytes of compressed image
        nr_bytes = struct.pack('<L', len(depth_img_comp))
        self.fp_d.write(nr_bytes)
        # write compressed image
        self.fp_d.write(depth_img_comp)
        self.fp_d.flush()


        if semantic_image is not None:
            cv2.imwrite(self.save_semantic_folder + str(step) + '_semantic.png', semantic_image)

    def record_motion_data(self, step, ts, gripper_pos, gripper_orn, gripper_linear, gripper_angular, gripper_state):

        #arr = [step, gripper_pos.x, gripper_pos.y, gripper_pos.z,
               #gripper_orn.w, gripper_orn.x, gripper_orn.y, gripper_orn.z,
               #gripper_linear.x, gripper_linear.y, gripper_linear.z,
               #gripper_angular.x, gripper_angular.y, gripper_angular.z, gripper_state, ts]

        arr = [step, ts, gripper_pos[0], gripper_pos[1], gripper_pos[2],
               gripper_orn[0], gripper_orn[1], gripper_orn[2], gripper_orn[3],
               gripper_linear[0], gripper_linear[1], gripper_linear[2],
               gripper_angular[0], gripper_angular[1], gripper_angular[2], gripper_state]

        self.writer.writerow(arr)

    def depth_from_binary(self, binary_name, imgsize=(120, 120)):
        """ Decode binary file containing depth images and return the depth
        images as a numpy ndarray.
        :param binary_name: The file name of the binary file to read.
        :param imgsize: The size (height, width) of each uncompressed image.
        :return: numpy array containing 'l' images of size 'imgsize'.
        """
        images = list()
        with open(binary_name, 'rb') as fp:
            b = fp.read(4)
            while (len(b) >= 4):
                k = struct.unpack('<L', b)[0]
                image_bytes = fp.read(k)
                images.append(snappy.uncompress(image_bytes))
                b = fp.read(4)
        l = len(images)
        images = np.array(images)
        images = np.frombuffer(images, dtype=np.float64)
        return images.reshape((l,) + (-1,))