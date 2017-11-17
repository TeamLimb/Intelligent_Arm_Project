#!/usr/bin/env python
import std_msgs, rospy

rospy.init_node('main_dealer', anonymous=True)
res_publisher = rospy.Publisher('result_detection', std_msgs.msg.Float64, queue_size=10)

import sys, os, cv2, time
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.backend import tensorflow_backend as K

dir_path = os.path.dirname(os.path.realpath(__file__))
pretrained_weight_path = os.path.join(dir_path, 'model_data/yolo-coco.h5')

sys.path.append(dir_path + "/model")
from tiny_yolo2 import tiny_yolo2, yolo_head, yolo_eval

LABELS = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
          'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
          'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
          'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
          'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
          'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
          'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
          'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
          'hair drier', 'toothbrush']
ANCHORS = [0.738768, 0.874946, 2.42204, 2.65704, 4.30971, 7.04493, 10.246, 4.59428, 12.6868, 11.8741]
ANCHORS = np.array(ANCHORS).reshape(-1, 2)
NUM_CLASS = len(LABELS)

OUR_DET = []
OUR_DET.append(LABELS.index('dog'))
OUR_DET.append(LABELS.index('bottle'))
OUR_DET.append(LABELS.index('sports ball'))

our_score = [0, 0, 0]

#os.system('sudo modprobe bcm2835-v4l2')
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

model = tiny_yolo2()
graph = tf.get_default_graph()
model.load_weights(pretrained_weight_path)

with graph.as_default():
    yolo_outputs = yolo_head(graph, model.output, ANCHORS, NUM_CLASS)
    input_image_shape = K.placeholder(shape=(2,))
    boxes, scores, classes = yolo_eval(graph, yolo_outputs, input_image_shape, score_threshold=0.1, iou_threshold=0.1)

sess = K.get_session()

def detection():
    with graph.as_default():
        t1 = time.time()

        ret, frame = cap.read()
        if not ret:
            print 'Not Found Devices.'
    	print 'Video capture successed.'

        frame = frame[32:448, 112:528, :]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        kernel = np.array(([0, -1, 0], [-1, 5, -1], [0, -1, 0]), dtype="int")
        frame = cv2.filter2D(frame, -1, kernel)

        frame = np.divide(frame, 255.)
        frame = np.expand_dims(frame, 0)

        t2 = time.time()
        print 'Video capture and preprocess takes {0} seconds.'.format(t2 - t1)

        t1 = time.time()

        out_boxes, out_scores, out_classes = sess.run(
            [boxes, scores, classes],
            feed_dict={
                model.input: frame,
                input_image_shape: [416, 416],
                K.learning_phase(): 0
            })

        t2 = time.time()

        print 'Detection takes {0} seconds.'.format(t2 - t1)
        return out_boxes, out_scores, out_classes

def deep_learning(execute):
	if execute.data == 1:
		best_match = -1
		best_score = -1

		_, o_scores, o_classes = detection()
		print o_scores
		print o_classes
		for i in range(len(o_classes)):
		    for j in range(len(OUR_DET)):
		        if o_classes[i] == OUR_DET[j]:
		            our_score[j] += o_scores[i]
		            if our_score[j] > best_score:
		                best_score = our_score[j]
		                best_match = j

                if best_match == -1:
                    res_publisher.publish(-1)
                else:
               	    res_publisher.publish(best_match + 1)

print 'Detection node is ready to process.'
rospy.Subscriber('exec_detection', std_msgs.msg.Float64, deep_learning)
rospy.spin()

sess.close()
