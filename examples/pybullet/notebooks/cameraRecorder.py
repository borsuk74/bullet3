import cv2 #as cv


class CameraRecorder(object):
    def __init__(self):
        """ Camera recorder class writing color  image messages recorded
        with the head camera of the baxter robot into a .avi video file and
        timestamps for each image frame into an accompanying .txt file.
        """      
        self._clip = None
        self._fp = None       
        self._running = False


    def clean_shutdown(self):
        """ Clean shutdown of the camera recorder. """

        self._close_clip()
        self._close_file()
        self._running = False

    def _close_clip(self):
        """ Close video file we wrote image frames into. """
        if self._clip:
            self._clip.release()
  

    def _close_file(self):
        """ Close text file we wrote time stamps into. """
        if self._fp:
            self._fp.close()
            self._fp = None


    def start(self, outname, fps, imgsize):
        """ Set up the camera recorder with the parameters for the recording
     
        :param outname: Filename to write the video and text file to, without
            the extension.
        :param fps: Frames per second for video file.
        :param imgsize: Size (width, height) of images to write into video
            file.
        :return: Whether the video- and text file were opened successfully.    
        """

        if not self._running:
                # open text file
                try:
                    self._fp = open(outname + '.txt', 'w')
                except IOError as e:
                    raise e
                self._fp.write('# timestamps [s]\n')
                # open video file
                self._clip = cv2.VideoWriter(outname + '.avi',
                                             fourcc=cv2.VideoWriter_fourcc('M','J','P','G'),
                                             fps=fps,
                                             frameSize=imgsize,
                                             isColor=True)
                if not self._clip.isOpened():
                    raise IOError("'%s' Failed to open videoWriter instance!" % self)

                self._running = True
        else:
                return self._clip.isOpened() and not self._fp.closed

   
    def add_image(self, ts, img):
        """ Camera subscriber callback function """
         #image in cv2 expected in format brg8
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

        self._fp.write('%f\n' % ts)
        self._fp.flush()
        # add frame to video
        try:
            self._clip.write(img)
        except Exception as e:
            raise e


    def getArrayFromVideo(self, file):
        capture = cv2.CaptureFromFile(file)

        frames = []

        for i in range(100):
            img = cv.QueryFrame(capture)

            tmp = cv.CreateImage(cv.GetSize(img), 8, 3)

            cv.CvtColor(img, tmp, cv.CV_BGR2RGB)

            frames.append(asarray(cv.GetMat(tmp)))

        frames = array(frames)
        return frames

    def getArrayFromVideo1(self, file):
        import os

        import scipy.io.wavfile as wav

        # install lame

        # install bleeding edge scipy (needs new cython)

        #fname = 'XC135672-Red-winged\ Blackbird1301.mp3'

        oname = 'temp.wav'

        cmd = 'lame --decode {0} {1}'.format(file, oname)

        os.system(cmd)

        data = wav.read(oname)

        # your code goes here

        #print(len(data[1]))
        return data

        #this one should be tested
    def getArrayFromVideo2(self, file):
        import skvideo.io
        video_data = skvideo.io.vread(file)
        print(video_data.shape)
        return video_data

