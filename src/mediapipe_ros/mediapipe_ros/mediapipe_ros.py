import time

import cv2
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from sensor_msgs.msg import Image

from mediapipe_ros_msgs.msg import BBox, BBoxArray


class MediapipeRos(Node):
    def __init__(self):
        super().__init__("mediapipe_ros")
        self.image_sub = self.create_subscription(Image, "image_raw", self.image_sub_cb, 1)
        self.bboxes_pub = self.create_publisher(BBoxArray, "~/bboxes", 10)
        self.image_pub = self.create_publisher(Image, "~/image", 10)

        self.declare_parameter("model", "./efficientdet.tflite")
        self.declare_parameter("max_results", 5)
        self.declare_parameter("score_threshold", 0.25)

        model = self.get_parameter("model").get_parameter_value().string_value
        max_results = self.get_parameter("max_results").get_parameter_value().integer_value
        score_threshold = self.get_parameter("score_threshold").get_parameter_value().double_value

        self.cv_bridge = CvBridge()

        # Initialize the object detection model
        self.detection_result_list = []
        base_options = python.BaseOptions(model_asset_path=model)
        options = vision.ObjectDetectorOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            max_results=max_results,
            score_threshold=score_threshold,
            result_callback=self._save_result,
        )
        self.detector = vision.ObjectDetector.create_from_options(options)

    def image_sub_cb(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

        # Run object detection using the model.
        self.detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        if self.detection_result_list:
            bboxes_msg = BBoxArray()

            for detection in self.detection_result_list[0].detections:
                bbox = detection.bounding_box
                category = detection.categories[0]

                bbox_msg = BBox()
                bbox_msg.name = category.category_name
                bbox_msg.score = category.score
                bbox_msg.xmin = bbox.origin_x
                bbox_msg.ymin = bbox.origin_y
                bbox_msg.xmax = bbox.origin_x + bbox.width
                bbox_msg.ymax = bbox.origin_y + bbox.height
                bboxes_msg.bboxes.append(bbox_msg)

                # Draw bounding_box
                cv2.rectangle(image, (bbox_msg.xmin, bbox_msg.ymin), (bbox_msg.xmax, bbox_msg.ymax), (255, 165, 0), 3)

                # Draw label and score
                cv2.putText(
                    image,
                    f"{bbox_msg.name} ({bbox_msg.score:.2f})",
                    (bbox.origin_x, bbox.origin_y),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

            self.detection_result_list.clear()
            self.bboxes_pub.publish(bboxes_msg)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, "rgb8"))

    def _save_result(self, result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        self.detection_result_list.append(result)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MediapipeRos()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
