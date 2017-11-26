from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
import os
import time
import math

from matplotlib import pyplot as plt
from PIL import Image

NUM_CLASSES=4

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        print(os.getcwd())
        self.ssd_mobilenet_model = 'light_classification/mobilenet_frozen_sim/frozen_inference_graph.pb'
        label_map = label_map_util.load_labelmap('light_classification/mobilenet_frozen_sim/label_map.pbtxt')
        
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
                
        self._detection_graph = tf.Graph()
  
        with self._detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(self.ssd_mobilenet_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')        

        self.sess =  tf.Session(graph=self._detection_graph)
        self.image_tensor = self._detection_graph.get_tensor_by_name('image_tensor:0')      
        print(self.image_tensor)
        print(self.category_index)            

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        with self._detection_graph.as_default():

            # Expand dimensions since the model expects
            # images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image, axis=0)
            image_tensor = self._detection_graph.get_tensor_by_name('image_tensor:0')
            # Bounding boxes for each object detection
            boxes = self._detection_graph.get_tensor_by_name('detection_boxes:0')
            # Confidence scores
            scores = self._detection_graph.get_tensor_by_name('detection_scores:0')
            classes = self._detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self._detection_graph.get_tensor_by_name('num_detections:0')

            start = time.time()

            # Detect
            (boxes, scores, classes, num_detections) = self.sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})

            end = time.time()
            #print (end-start)

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            
            #vis_util.visualize_boxes_and_labels_on_image_array(
            #    image, boxes, classes, scores,
            #    self.category_index,
            #    use_normalized_coordinates=True,
            #    line_thickness=6)
            
            # IMAGE_SIZE = (12, 8)
            # plt.figure(figsize=IMAGE_SIZE)
            # plt.imshow(image)
            # plt.show()

            min_score_thresh = .80

            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > min_score_thresh:

                    class_name = self.category_index[classes[i]]['name']
                    perceived_width_x = (boxes[i][3] - boxes[i][1])*800 
                    perceived_width_y = (boxes[i][2] - boxes[i][0])*600
                    diagonale = math.sqrt(perceived_width_x*perceived_width_y)
                    print('TL_Classifier: {}'.format(class_name), scores[i], diagonale)

                    # return element with the highest score
                    if class_name == 'Red':
                        return TrafficLight.RED
                    elif class_name == 'Yellow':
                        return  TrafficLight.YELLOW
                    elif class_name == 'Green':
                        return  TrafficLight.GREEN

        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
