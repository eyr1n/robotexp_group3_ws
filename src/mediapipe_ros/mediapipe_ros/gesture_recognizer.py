import time

import cv2
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from sensor_msgs.msg import Image

from mediapipe_ros_msgs.msg import Gesture, GestureArray


class GestureRecognizer(Node):
    def __init__(self):
        super().__init__("gesture_recognizer")

        self.image_sub = self.create_subscription(Image, "image_raw", self.image_sub_cb, 1)
        self.gestures_pub = self.create_publisher(GestureArray, "~/gestures", 10)
        self.image_pub = self.create_publisher(Image, "~/image", 10)

        self.declare_parameter("model", "./gesture_recognizer.task")
        self.declare_parameter("num_hands", 1)
        self.declare_parameter("min_hand_detection_confidence", 0.5)
        self.declare_parameter("min_hand_presence_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        model = self.get_parameter("model").get_parameter_value().string_value
        num_hands = self.get_parameter("num_hands").get_parameter_value().integer_value
        min_hand_detection_confidence = (
            self.get_parameter("min_hand_detection_confidence").get_parameter_value().double_value
        )
        min_hand_presence_confidence = (
            self.get_parameter("min_hand_presence_confidence").get_parameter_value().double_value
        )
        min_tracking_confidence = self.get_parameter("min_tracking_confidence").get_parameter_value().double_value

        self.cv_bridge = CvBridge()

        # Initialize the gesture recognizer model
        self.recognition_result_list = []
        base_options = python.BaseOptions(model_asset_path=model)
        options = vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_hands=num_hands,
            min_hand_detection_confidence=min_hand_detection_confidence,
            min_hand_presence_confidence=min_hand_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
            result_callback=self.save_result,
        )
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

    def image_sub_cb(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

        # Run gesture recognizer using the model.
        self.recognizer.recognize_async(mp_image, time.time_ns() // 1_000_000)

        # Draw the hand landmarks.
        if self.recognition_result_list:
            # Draw landmarks.
            for hand_landmarks in self.recognition_result_list[0].hand_landmarks:
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend(
                    [
                        landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z)
                        for landmark in hand_landmarks
                    ]
                )

                mp.solutions.drawing_utils.draw_landmarks(
                    image,
                    hand_landmarks_proto,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style(),
                )

            gestures_msg = GestureArray()

            for i, gesture in enumerate(self.recognition_result_list[0].gestures):
                gesture_msg = Gesture()
                gesture_msg.name = gesture[0].category_name
                gesture_msg.score = gesture[0].score
                gestures_msg.gestures.append(gesture_msg)

                # Draw the text
                cv2.putText(
                    image,
                    f"{gesture_msg.name} ({gesture_msg.score:.2f})",
                    (20, 20 * (i + 1)),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

            self.recognition_result_list.clear()
            self.gestures_pub.publish(gestures_msg)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, "rgb8"))

    def save_result(self, result: vision.GestureRecognizerResult, unused_output_image: mp.Image, timestamp_ms: int):
        self.recognition_result_list.append(result)


def main(args=None):
    rclpy.init(args=args)
    gesture_recognizer = GestureRecognizer()
    rclpy.spin(gesture_recognizer)
    gesture_recognizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
