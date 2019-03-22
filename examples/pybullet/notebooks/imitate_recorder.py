import time
import os
import csv
import sys
import itertools
import numpy as np
import cv2

class ImitationRecorder():
    def __init__(self, task):
        # Create the appropriate directory in the datas for the task we are training
        if not os.path.exists('../datas/' + task + '/'):
            os.mkdir('../datas/' + task + '/')
        self.save_folder = None  # The specific folder
        self.writer = None  # The writer to create our txt files
        self.text_file = None  # The file that we are writing to currently
        self.is_recording = False  # Toggling recording
        self.task = task

    def toggle_recording(self, toggle):
        if toggle is "0":
            self.is_recording = False

            if self.text_file != None:
                self.text_file.close()
                self.text_file = None
            print("-----Stop Recording-----")
        else:
            save_folder = '../datas/' + self.task + '/' + str(time.time()) + '/'
            os.mkdir(save_folder)
            self.save_folder = save_folder
            self.text_file = open(save_folder + 'vectors.txt', 'w')
            self.writer = csv.writer(self.text_file)
            self.is_recording = True
            print("=====Start Recording=====")

    def record_video_data(self, step, rgb_image, depth_image, semantic_image):
        cv2.imwrite(self.save_folder + str(step) + '_rgb.png', rgb_image)
        cv2.imwrite(self.save_folder + str(step) + '_depth.png', depth_image)
        cv2.imwrite(self.save_folder + str(step) + '_semantic.png', semantic_image)

    def record_motion_data(self, step, ts, gripper_pos, gripper_orn, gripper_linear, gripper_angular, gripper_state):

        arr = [step, gripper_pos.x, gripper_pos.y, gripper_pos.z,
               gripper_orn.w, gripper_orn.x, gripper_orn.y, gripper_orn.z,
               gripper_linear.x, gripper_linear.y, gripper_linear.z,
               gripper_angular.x, gripper_angular.y, gripper_angular.z, gripper_state, ts]

        self.writer.writerow(arr)
