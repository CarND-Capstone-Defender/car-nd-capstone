import rospy
from matplotlib import pyplot as plt      
from styx_msgs.msg import TrafficLight
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
import os
import time
import math
import yaml
import cv2

#for image publishing
from sensor_msgs.msg import CompressedImage


NUM_CLASSES = 4
MIN_DETECTION_PROB_THRESHOLD = 0.6

class TLClassifier(object):

    def __init__(self):
        rospy.logdebug(os.getcwd())

        self.bridge = CvBridge()

        self.mode = self.getMode()
        if (self.mode == "SIM" or self.mode == "SIM_CHURCHLOT"):
            # assume we are in simulator mode
            rospy.logdebug('Assuming SIM mode...try to load mobilenet_frozen_sim')
            self.mode = "SIM" 
            self.ssd_mobilenet_model = 'light_classification/mobilenet_frozen_sim/frozen_inference_graph.pb'
            label_map = label_map_util.load_labelmap('light_classification/mobilenet_frozen_sim/label_map.pbtxt')
            rospy.logdebug('Loading model successfully from light_classification/mobilenet_frozen_sim/frozen_inference_graph.pb')
            rospy.logdebug('Loading label map successfully from light_classification/mobilenet_frozen_sim/label_map.pbtxt')
        else:
            # assume we are in Carla mode
            rospy.logdebug('Assuming Real-Drive mode... try to load mobilenet_frozen_real')
            self.mode = "CARLA" 
            self.ssd_mobilenet_model = 'light_classification/mobilenet_frozen_real/frozen_inference_graph.pb'
            label_map = label_map_util.load_labelmap('light_classification/mobilenet_frozen_real/label_map.pbtxt')
            rospy.logdebug('Loading model successfully from light_classification/mobilenet_frozen_real/frozen_inference_graph.pb')
            rospy.logdebug('Loading label map successfully from light_classification/mobilenet_frozen_real/label_map.pbtxt')

        
        # Loading the inference graph
        self._detection_graph = tf.Graph()
        with self._detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(self.ssd_mobilenet_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')        
                rospy.logdebug('Loading of serialized inference graph was successful')
        
        rospy.logdebug('Now starting Tensorflow session...')
        
        # Starting Tensorflow session
        self.sess =  tf.Session(graph=self._detection_graph)
        self.image_tensor = self._detection_graph.get_tensor_by_name('image_tensor:0')      

        rospy.logdebug('Tensorflow successfully started...')
        rospy.logdebug('image_tensor:0 = %s' ,  self.image_tensor)

        # Loading label map categories
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        rospy.logdebug('Label Map = %s'  , self.category_index)          

        self.publish_trafficlight_image = rospy.Publisher("/tl_classifier/image_raw/compressed", CompressedImage, queue_size=4)      
        rospy.logdebug('Publishing ROS message /tl_classifier/image_raw/compressed')          
                
    def getMode(self):
        speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s
       
        # CARLA track has speed limit 10
        if (speed_limit > 10):
            # assume we are in simulator mode
            rospy.logdebug("Speed limit < 10 - assuming simulator mode")
            mode = "SIM" 
        else:
            # assume we are in Carla mode - but is it simulator churchlot or real drive!?
            is_site_configured = rospy.has_param("/grasshopper_calibration_yaml")

            if is_site_configured == False:
                rospy.logdebug("No grasshopper found - assuming simulator churchlot mode")
                mode = "SIM_CHURCHLOT" 
            else:
                rospy.logdebug("Grasshopper found - assuming real-drive mode")
                mode = "CARLA" 
        
        rospy.logdebug("Identified mode %s" , mode)        
        return mode        
               

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        rospy.logdebug('tl_classifier.get_classification() called')

        if self._detection_graph.as_default() == None:
            rospy.logerror('Error: self._detection_graph.as_default() is None')
            return TrafficLight.UNKNOWN

        # Preprocess the image 
        if self.mode == "SIM":
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) 
            rospy.logdebug('Converting image from BGR to RGB in SIM mode')
        else:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) 
            rospy.logdebug('Converting image from BGR to RGB in CALRA mode')
            pass

        with self._detection_graph.as_default():

            # Expand dimensions since the model expects
            # images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(cv_image, axis=0)
            image_tensor = self._detection_graph.get_tensor_by_name('image_tensor:0')
            # Get bounding boxes for each object detection
            boxes = self._detection_graph.get_tensor_by_name('detection_boxes:0')
            # Get confidence scores
            scores = self._detection_graph.get_tensor_by_name('detection_scores:0')
            classes = self._detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self._detection_graph.get_tensor_by_name('num_detections:0')

            rospy.logdebug('Starting image detection...')
            start = time.time()

            # Feed dictionary and start tensorflow session for detection
            (boxes, scores, classes, num_detections) = self.sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})

            rospy.logdebug('Ending image detection...')
            end = time.time()
            rospy.logdebug('Time needed for detection in milliseconds: %s' , int(round((end-start)*1000,0)))


            # Finally process the detection results
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            
            # Annotate the image and publish as topic /tl_classifier/image_raw/compressed
            vis_util.visualize_boxes_and_labels_on_image_array(
            cv_image, boxes, classes, scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=6)
            
            # Create CompressedIamge #
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            # Publish new image
            self.publish_trafficlight_image.publish(msg)
            
            
            # loop through all bounding boxes which have been found
            for i in range(boxes.shape[0]):
                # only loop through bounding boxes which score is higher 
                # than the minimal threshold MIN_DETECTION_PROB_THRESHOLD
                if scores is not None and scores[i] > MIN_DETECTION_PROB_THRESHOLD:
                    class_name = self.category_index[classes[i]]['name']
                    perceived_width_x = (boxes[i][3] - boxes[i][1])*800 
                    perceived_width_y = (boxes[i][2] - boxes[i][0])*600
                    diagonal = math.sqrt(perceived_width_x*perceived_width_y)
                    rospy.logdebug('TL_Classifier: Color = %s  , Probability = %s' , class_name , round(scores[i],2))
                    rospy.logdebug('TL_Classifier: Diagonal of Bounding box = %s' , round(diagonal,0))


                    # immediately return the detection with the highest score
                    # other detections are ignored
                    if class_name == 'Red':
                        return TrafficLight.RED
                    elif class_name == 'Yellow':
                        return  TrafficLight.YELLOW
                    elif class_name == 'Green':
                        return  TrafficLight.GREEN

            
            rospy.logdebug('No detection results found...')

        return TrafficLight.UNKNOWN
