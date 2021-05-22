import os
import cv2
import numpy
import tensorflow
import rospy
from styx_msgs.msg import TrafficLight

class TLClassifier(object):

    def __init__(self):
        self.MIN_SCORE_THRESHOLD = 0.4
        PATH_TO_CKPT = os.path.join(os.getcwd(), 'light_classification', 'frozen_inference_graph.pb')
        detection_graph = tensorflow.Graph()
        with detection_graph.as_default():
            graph_def = tensorflow.GraphDef()
            with tensorflow.gfile.GFile(PATH_TO_CKPT, 'rb') as gfile:
                graph_def.ParseFromString(gfile.read())
                tensorflow.import_graph_def(graph_def, name = '')
            self.sess = tensorflow.Session(graph = detection_graph)
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.params = \
            [detection_graph.get_tensor_by_name('detection_boxes:0'),
            detection_graph.get_tensor_by_name('detection_scores:0'), 
            detection_graph.get_tensor_by_name('detection_classes:0'), 
            detection_graph.get_tensor_by_name('num_detections:0')]
        rospy.loginfo('TRAFFIC LIGHT CLASSIFIER WAS LOADED SUCCESSFULLY.')
        
    def detect_objects(self, image):
        image_dict = {self.image_tensor: numpy.expand_dims(image, axis=0)}
        return self.sess.run(self.params, feed_dict = image_dict)

    def get_classification(self, image):
        boxes, scores, classes, num_detections = self.detect_objects(image)
        c = classes[0][0]
        if scores[0][0] < self.MIN_SCORE_THRESHOLD:
            light = TrafficLight.UNKNOWN
        elif c == 1:
            light = TrafficLight.RED
        elif c == 2:
            light = TrafficLight.YELLOW
        elif c == 3:
            light = TrafficLight.GREEN            
        else:
            light = TrafficLight.UNKNOWN
        return light