import tensorflow as tf
from keras.backend import tensorflow_backend as K
from keras.models import Model
from keras.layers import Activation, Conv2D, MaxPooling2D, BatchNormalization, Input
from keras.layers.advanced_activations import LeakyReLU
from keras import regularizers

def tiny_yolo2():
    my_input = Input(shape=(416, 416, 3))
    output = Conv2D(16, (3, 3),
                    strides=(1, 1),
                    padding='same',
                    use_bias=False,
                    kernel_initializer='glorot_normal',
                    kernel_regularizer=regularizers.l2(0.01))(my_input)
    output = BatchNormalization(beta_regularizer=regularizers.l2(0.01), gamma_regularizer=regularizers.l2(0.01))(output)
    output = LeakyReLU(alpha=0.1)(output)
    output = MaxPooling2D(pool_size=(2, 2))(output)

    # Layer 2 - 5
    for i in range(0, 4):
        output = Conv2D(32 * (2 ** i), (3, 3),
                        strides=(1, 1),
                        padding='same',
                        use_bias=False,
                        kernel_initializer='glorot_normal',
                        kernel_regularizer=regularizers.l2(0.01))(output)
        output = BatchNormalization(beta_regularizer=regularizers.l2(0.01), gamma_regularizer=regularizers.l2(0.01))(output)
        output = LeakyReLU(alpha=0.1)(output)
        output = MaxPooling2D(pool_size=(2, 2))(output)

    # Layer 6
    output = Conv2D(512, (3, 3),
                    strides=(1, 1),
                    padding='same',
                    use_bias=False,
                    kernel_initializer='glorot_normal',
                    kernel_regularizer=regularizers.l2(0.01))(output)
    output = BatchNormalization(beta_regularizer=regularizers.l2(0.01), gamma_regularizer=regularizers.l2(0.01))(output)
    output = LeakyReLU(alpha=0.1)(output)
    output = MaxPooling2D(pool_size=(2, 2), strides=(1, 1), padding='same')(output)

    # Layer 7 - 8
    for _ in range(0, 2):
        output = Conv2D(1024, (3, 3),
                        strides=(1, 1),
                        padding='same',
                        use_bias=False,
                        kernel_initializer='glorot_normal',
                        kernel_regularizer=regularizers.l2(0.01))(output)
        output = BatchNormalization(beta_regularizer=regularizers.l2(0.01), gamma_regularizer=regularizers.l2(0.01))(output)
        output = LeakyReLU(alpha=0.1)(output)

    # Layer 9
    output = Conv2D(5 * (4 + 1 + 80), (1, 1),
                    strides=(1, 1),
                    padding='same',
                    kernel_initializer='glorot_normal',
                    kernel_regularizer=regularizers.l2(0.01))(output)
    output = Activation('linear')(output)

    return Model(my_input, output)

def yolo_head(graph, feats, anchors, num_classes):
    with graph.as_default():
        num_anchors = len(anchors)
        anchors_tensor = K.reshape(K.variable(anchors), [1, 1, 1, num_anchors, 2])

        conv_dims = K.shape(feats)[1:3]
        conv_height_index = K.arange(0, stop=conv_dims[0])
        conv_width_index = K.arange(0, stop=conv_dims[1])
        conv_height_index = K.tile(conv_height_index, [conv_dims[1]])

        conv_width_index = K.tile(K.expand_dims(conv_width_index, 0), [conv_dims[0], 1])
        conv_width_index = K.flatten(K.transpose(conv_width_index))
        conv_index = K.transpose(K.stack([conv_height_index, conv_width_index]))
        conv_index = K.reshape(conv_index, [1, conv_dims[0], conv_dims[1], 1, 2])
        conv_index = K.cast(conv_index, K.dtype(feats))

        feats = K.reshape(feats, [-1, conv_dims[0], conv_dims[1], num_anchors, num_classes + 5])
        conv_dims = K.cast(K.reshape(conv_dims, [1, 1, 1, 1, 2]), K.dtype(feats))

        box_xy = K.sigmoid(feats[..., :2])
        box_wh = K.exp(feats[..., 2:4])
        box_confidence = K.sigmoid(feats[..., 4:5])
        box_class_probs = K.softmax(feats[..., 5:])

        box_xy = (box_xy + conv_index) / conv_dims
        box_wh = box_wh * anchors_tensor / conv_dims

        return box_xy, box_wh, box_confidence, box_class_probs

def yolo_boxes_to_corners(graph, box_xy, box_wh):
    with graph.as_default():
        box_mins = box_xy - (box_wh / 2.)
        box_maxes = box_xy + (box_wh / 2.)

        return K.concatenate([
            box_mins[..., 1:2],  # y_min
            box_mins[..., 0:1],  # x_min
            box_maxes[..., 1:2],  # y_max
            box_maxes[..., 0:1]  # x_max
        ])

def yolo_filter_boxes(graph, boxes, box_confidence, box_class_probs, threshold=.6):
    with graph.as_default():
        box_scores = box_confidence * box_class_probs
        box_classes = K.argmax(box_scores, axis=-1)
        box_class_scores = K.max(box_scores, axis=-1)
        prediction_mask = box_class_scores >= threshold

        boxes = tf.boolean_mask(boxes, prediction_mask)
        scores = tf.boolean_mask(box_class_scores, prediction_mask)
        classes = tf.boolean_mask(box_classes, prediction_mask)
        return boxes, scores, classes

def yolo_eval(graph, yolo_outputs, image_shape, max_boxes=10, score_threshold=.6, iou_threshold=.5):
    with graph.as_default():
        box_xy, box_wh, box_confidence, box_class_probs = yolo_outputs
        boxes = yolo_boxes_to_corners(graph, box_xy, box_wh)
        boxes, scores, classes = yolo_filter_boxes(graph, boxes, box_confidence, box_class_probs, threshold=score_threshold)

        # Scale boxes back to original image shape.
        height = image_shape[0]
        width = image_shape[1]
        image_dims = K.stack([height, width, height, width])
        image_dims = K.reshape(image_dims, [1, 4])
        boxes = boxes * image_dims

        # TODO: Something must be done about this ugly hack!
        max_boxes_tensor = K.variable(max_boxes, dtype='int32')
        K.get_session().run(tf.variables_initializer([max_boxes_tensor]))
        nms_index = tf.image.non_max_suppression(boxes, scores, max_boxes_tensor, iou_threshold=iou_threshold)
        boxes = K.gather(boxes, nms_index)
        scores = K.gather(scores, nms_index)
        classes = K.gather(classes, nms_index)

        return boxes, scores, classes

