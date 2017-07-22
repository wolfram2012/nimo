from __future__ import print_function
import mxnet as mx
import numpy as np
from timeit import default_timer as timer
import cv2
from tools.rand_sampler import RandSampler

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from coms_msgs.msg import ACCCommand
from cv_bridge import CvBridge

import numpy as np
import cv2
import pyvision as pv
import ocof.filters.common as common
import ocof
import time

cap = cv2.VideoCapture(0)
ret,imgi = cap.read()
ret,img_rect = cap.read()
colors = dict()

global tracker
tracker = None
global TrackFlag
TrackFlag = False
global TAZ_RECT


class DetIter(mx.io.DataIter):
   
    def __init__(self, batch_size, data_shape, \
                 mean_pixels=[128, 128, 128], rand_samplers=[], \
                 rand_mirror=False, shuffle=False, rand_seed=None, \
                 is_train=True, max_crop_trial=50):
        super(DetIter, self).__init__()

        self.batch_size = batch_size
        if isinstance(data_shape, int):
            data_shape = (data_shape, data_shape)
        self._data_shape = data_shape
        self._mean_pixels = mx.nd.array(mean_pixels).reshape((3,1,1))
        # if not rand_samplers:
        #     self._rand_samplers = []
        # else:
        #     if not isinstance(rand_samplers, list):
        #         rand_samplers = [rand_samplers]
        #     assert isinstance(rand_samplers[0], RandSampler), "Invalid rand sampler"
        #     self._rand_samplers = rand_samplers
        self._rand_mirror = rand_mirror
        self._shuffle = shuffle
        if rand_seed:
            np.random.seed(rand_seed) # fix random seed
        self._max_crop_trial = max_crop_trial

        self._current = 0
        self._size = 1
        self._index = np.arange(self._size)

        self._data = None
        self._label = None
        self._get_batch()

    def reset(self):
        self._current = 0
        if self._shuffle:
            np.random.shuffle(self._index)

    def iter_next(self):
        return self._current < self._size

    def next(self):
        if self.iter_next():
            self._get_batch()
            data_batch = mx.io.DataBatch(data=self._data.values(),
                                   label=self._label.values(),
                                   pad=self.getpad(), index=self.getindex())
            self._current += self.batch_size
            return data_batch
        else:
            raise StopIteration

    def getindex(self):
        return self._current // self.batch_size

    def getpad(self):
        pad = self._current + self.batch_size - self._size
        return 0 if pad < 0 else pad

    def _get_batch(self):
        """
        Load data/label from dataset
        """
        # index = self._index[self._current]
        # im_path = self._imdb.image_path_from_index(0)
        # im_path = 'data/demo/dog.jpg'
        # with open(im_path, 'rb') as fp:
        #     img_content = fp.read()

        batch_data = mx.nd.zeros((self.batch_size, 3, self._data_shape[0], self._data_shape[1]))
        batch_label = [] 
        global imgi
        # img = mx.nd.array(imgi)
        # imgr = mx.img.imdecode(img_content)
        data = self._data_augmentation(imgi)
        batch_data[0] = data
            
        self._data = {'data': batch_data}
        self._label = {'label': None}

    def _data_augmentation(self, data):
        """
        perform data augmentations: crop, mirror, resize, sub mean, swap channels...
        """
        data = mx.img.imresize(data, self._data_shape[1], self._data_shape[0], cv2.INTER_LINEAR)
        data = mx.nd.transpose(data, (2,0,1))
        data = data.astype('float32')
        data = data - self._mean_pixels
        return data



class Detector(object):
    
    def __init__(self, symbol, model_prefix, epoch, data_shape, mean_pixels, \
                 batch_size=1, ctx=None):
        self.ctx = ctx
        load_symbol, args, auxs = mx.model.load_checkpoint(model_prefix, epoch)
        self.mod = mx.mod.Module(symbol, label_names=None, context=ctx)
        self.data_shape = data_shape
        self.mod.bind(data_shapes=[('data', (batch_size, 3, data_shape, data_shape))])
        self.mod.set_params(args, auxs)
        self.data_shape = data_shape
        self.mean_pixels = mean_pixels

    def detect(self, det_iter, show_timer=False):
        
        num_images = det_iter._size
        # if not isinstance(det_iter, mx.io.PrefetchingIter):
        #     det_iter = mx.io.PrefetchingIter(det_iter)
        start = timer()
        detections = self.mod.predict(det_iter).asnumpy()
        time_elapsed = timer() - start
        if show_timer:
            print("Detection time for {} images: {:.4f} sec".format(
                num_images, time_elapsed))
        result = []
        for i in range(detections.shape[0]):
            det = detections[i, :, :]
            res = det[np.where(det[:, 0] >= 0)[0]]
            result.append(res)
        return result

    def im_detect(self, root_dir=None, extension=None, show_timer=False):
        
        # test_db = TestDB(im_list, root_dir=root_dir, extension=extension)
        test_iter = DetIter( 1, self.data_shape, self.mean_pixels,
                            is_train=False)
        return self.detect(test_iter, show_timer)

    def randomRGB(self):
        import random
        return int(233 *random.random())

    def colorInv(self,color):
        return (255-color[0], 255-color[1], 255-color[1])

    def visualize_detection(self, img, dets, classes=[], thresh=0.6):

        height = img.shape[0]
        width = img.shape[1]
        global colors
        global img_depth
        global trackpos
        # print dets.shape[0]
        acc_flag = False
        for i in range(dets.shape[0]):
            cls_id = int(dets[i, 0])
            # if cls_id >= 0:
            
            if cls_id == 14:
                score = dets[i, 1]
                if score > thresh:
                    if cls_id not in colors:
                        colors[cls_id] = (self.randomRGB(), self.randomRGB(), self.randomRGB())
                    if trackpos == 0:
                        trackpos =  int(0.5*(int(dets[i, 4] * width)+int(dets[i, 2] * width)))
                    if abs(trackpos - (int(0.5*(int(dets[i, 4] * width)+int(dets[i, 2] * width))))) < 30:
                        xmin = int(dets[i, 2] * width)
                        ymin = int(dets[i, 3] * height)
                        xmax = int(dets[i, 4] * width)
                        ymax = int(dets[i, 5] * height)
                        img_roi = img_depth[ymin:ymax,xmin:xmax]
                        temp_height = img_roi.shape[0] 
                        temp_width = img_roi.shape[1]
                        # print("high:{} width:{} ".format(height, width ))
                        # print("xmin:{} xmax:{} ymin:{} ymax:{}".format(xmin, xmax, ymin, ymax))
                        biggest = np.amin(img_roi)
                        required = (img_roi[int(0.5*(ymax-ymin)),int(0.5*(xmax-xmin))])
                        # global TAZ_RECT
                        trackpos = int(0.5*(xmax+xmin))
                        acc_flag = True

                        global tracker
                        TAZ_RECT = pv.Rect(xmin, ymax, xmax-xmin, ymax-ymin)
                        uFrame = pv.Image(img.copy())
                        tracker = ocof.MOSSETrack(uFrame,TAZ_RECT)

                        global acc_pub
                        accCmd = ACCCommand()
                        accCmd.logtitudeErr = required
                        accCmd.lateralErr = trackpos
                        acc_pub.publish(accCmd)

                        cv2.rectangle(
                            img, (xmin, ymin), (xmax, ymax), color=colors[cls_id])
                        class_name = str(cls_id)
                        if classes and len(classes) > cls_id:
                            class_name = classes[cls_id]
                        # text = '{:s} {:.2f} {:.2f}'.format(class_name, score, required)
                        text = '{:s} {:.2f}m'.format(class_name, required)
                        texSize, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 1, 1)
                        cv2.rectangle(
                            img, (xmin, ymin - 2), (xmin + texSize[0], ymin - texSize[1]), colors[cls_id], -1)
                        cv2.putText(img, text,
                                    (xmin, ymin - 2), cv2. FONT_HERSHEY_DUPLEX, 1, self.colorInv(colors[cls_id]), 1)

        acc_enable.publish(acc_flag)
        # cv2.namedWindow("stra")
        cv2.imshow("stra",img)
        # cv2.waitKey()

    def ImgCcallback(self, ros_img):
        global imgi
        imgi = mx.img.imdecode(ros_img.data)
        global img_rect
        global img_raw
        global Frame
        img_rect = CvBridge().compressed_imgmsg_to_cv2(ros_img)
        # img_raw = img_rect.copy()

        Frame = pv.Image(img_rect.copy())
        # cv2.imshow("tester", Frame.asOpenCV())

    # def Imgcallback(self, ros_img):
    #     global img_rect
    #     img_rect = CvBridge().imgmsg_to_cv2(ros_img)


    def DepthImgcallback(self, ros_img):
        global img_depth
        img_depth = CvBridge().imgmsg_to_cv2(ros_img)
        # cv2.imshow("tester", img_depth)


    # def trackCallback(event):
    #     print "1"

    def trackCallback(self, event):
        ikkk = event.current_real
        # print("event: {}".format(ikkk))
        # print("im here")
        global Frame
        global img_raw
        global tracker
        global show
        global imshow

        if tracker is None:
            print("track none")
        # if TrackFlag is True:
        else:
            uframe = Frame
            start = time.time()
            tracker.update(uframe)
            stop = time.time()
            print("update time: {}".format(stop-start))
            rect = tracker.rect
            # print("shape:{}".format(rect))
            show = uframe.asOpenCV()
            # print("{}".format(show.shape))
            cv2.rectangle(show, ((int)(rect.x),(int)(rect.y-rect.h)), ((int)(rect.x+rect.w), (int)(rect.y)), (255,0,0),2)
            imshow = 1
            # cv2.imshow("tester", show)
            # print("track arrive")
            # cv2.waitKey()
            
    def detect_and_visualize(self, root_dir=None, extension=None,
                             classes=[], thresh=0.6, show_timer=False):
        
        global imgi
        global img_rect
        global Frame
        global show
        global trackpos
        global imshow
        global acc_pub
        global acc_enable

        acc_pub = rospy.Publisher("acc_cmd", ACCCommand, queue_size=2)
        acc_enable = rospy.Publisher("acc_enable", Bool, queue_size=2)
        # rospy.Timer(rospy.Duration(0.02), self.trackCallback)
        rospy.Subscriber("/zed/left/image_rect_color/compressed",CompressedImage, self.ImgCcallback,  queue_size = 4)
        # rospy.Subscriber("/zed/left/image_rect_color",Image, self.Imgcallback,  queue_size = 4)
        rospy.Subscriber("/zed/depth/depth_registered",Image, self.DepthImgcallback,  queue_size = 4)
        im_path = '/home/wolfram/mxnet/example/ssd/data/demo/dog.jpg'
        with open(im_path, 'rb') as fp:
            img_content = fp.read()

        trackpos = 0
        imshow = 0
        imgi = mx.img.imdecode(img_content)
        while(1):
            
            # ret,img_rect = cap.read()
            dets = self.im_detect(root_dir, extension, show_timer=show_timer)
            # if not isinstance(im_list, list):
            #     im_list = [im_list]
            # assert len(dets) == len(im_list)
            # for k, det in enumerate(dets):
        
            # img[:, :, (0, 1, 2)] = img[:, :, (2, 1, 0)]
            # img = img_rect.copy()
            self.visualize_detection(img_rect, dets[0], classes, thresh)

            if imshow == 1:
                cv2.imshow("tester", show)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
