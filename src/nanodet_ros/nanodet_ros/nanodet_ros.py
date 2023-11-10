import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from nanodet.util import cfg, load_config
from nanodet_ros_msgs.msg import BBox, BBoxArray

from .predictor import Predictor


class NanodetRos(Node):
    def __init__(self):
        super().__init__("nanodet_ros")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, "image_raw/compressed", self.camera_callback, 1)

        load_config(cfg, "./nanodet/config/legacy_v0.x_configs/nanodet-m.yml")
        self.predictor = Predictor(cfg, "./nanodet/nanodet_m.ckpt")

        self.publisher = self.create_publisher(BBoxArray, "~/bboxes", 10)

    def camera_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        res = self.predictor.inference(frame)
        # self.predictor.visualize(res[0], frame, cfg.class_names, 0.3)
        # cv2.waitKey(1)
        bboxes = BBoxArray()
        dets = res[0]
        for label in dets:
            for bbox in dets[label]:
                score = bbox[-1]
                if score >= 0.3:
                    bbox = bbox[:-1]
                    clsname = cfg.class_names[label]
                    bboxmsg = BBox()
                    bboxmsg.name = clsname
                    bboxmsg.score = score
                    bboxmsg.xmin = bbox[0]
                    bboxmsg.ymin = bbox[1]
                    bboxmsg.xmax = bbox[2]
                    bboxmsg.ymax = bbox[3]
                    bboxes.bboxes.append(bboxmsg)
        self.publisher.publish(bboxes)


def main():
    rclpy.init()
    node = NanodetRos()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
