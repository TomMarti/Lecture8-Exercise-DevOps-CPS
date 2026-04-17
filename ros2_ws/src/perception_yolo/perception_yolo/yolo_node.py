#!/usr/bin/env python3
"""YOLO object detection ROS 2 node.

Subscribes: sensor_msgs/Image (raw RGB camera stream)
Publishes:  perception_msgs/DetectionArray (bounding boxes + classes + confidence)
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from perception_msgs.msg import Detection, DetectionArray


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # Parameters — all overridable via --ros-args -p
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("input_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")  # or 'cuda:0'
        self.declare_parameter("annotated_topic", "/perception/image_annotated")
        self.declare_parameter("publish_annotated", True)

        model_name = self.get_parameter("model").value
        input_topic = self.get_parameter("input_topic").value
        det_topic = self.get_parameter("detections_topic").value
        self.conf_thr = self.get_parameter("conf_threshold").value
        self.iou_thr = self.get_parameter("iou_threshold").value
        self.device = self.get_parameter("device").value

        self.get_logger().info(f"Loading YOLO model: {model_name} on {self.device}")
        self.model = YOLO(model_name)  # auto-downloads on first use
        self.names = self.model.names

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, input_topic, self.on_image, 10)
        self.pub = self.create_publisher(DetectionArray, det_topic, 10)

        annot_topic = self.get_parameter("annotated_topic").value
        self.publish_annotated = self.get_parameter("publish_annotated").value
        self.annot_pub = (
            self.create_publisher(Image, annot_topic, 10)
            if self.publish_annotated
            else None
        )

        # lightweight perf counters
        self._n_frames = 0
        self._t_window_start = time.time()
        self._last_inf_ms = 0.0

        self.get_logger().info(
            f"Ready. Sub: {input_topic} "
            f"-> Pub dets: {det_topic} "
            f"| Pub img: {annot_topic if self.publish_annotated else 'off'} "
            f"(conf>={self.conf_thr}, iou>={self.iou_thr})"
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        t0 = time.perf_counter()
        results = self.model.predict(
            source=frame,
            conf=self.conf_thr,
            iou=self.iou_thr,
            device=self.device,
            verbose=False,
        )
        self._last_inf_ms = (time.perf_counter() - t0) * 1000.0

        out = DetectionArray()
        out.header = msg.header
        out.image_height, out.image_width = frame.shape[:2]

        r = results[0]
        if r.boxes is not None and len(r.boxes) > 0:
            # pull everything off GPU in one go for efficiency
            xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clss = r.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), cf, ci in zip(xyxy, confs, clss):
                d = Detection()
                d.class_id = int(ci)
                d.class_name = str(self.names.get(int(ci), "unknown"))
                d.confidence = float(cf)
                d.bbox_x = float(x1)
                d.bbox_y = float(y1)
                d.bbox_width = float(x2 - x1)
                d.bbox_height = float(y2 - y1)
                out.detections.append(d)

        self.pub.publish(out)

        # Annotated image stream
        if self.annot_pub is not None:
            # results[0].plot() draws boxes + labels + confidences in BGR.
            # Returns a fresh ndarray — does not mutate `frame`.
            annotated = r.plot()
            annot_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annot_msg.header = msg.header   # preserve original frame time + id
            self.annot_pub.publish(annot_msg)

        self._n_frames += 1
        now = time.time()
        if now - self._t_window_start >= 5.0:
            fps = self._n_frames / (now - self._t_window_start)
            self.get_logger().info(
                f"FPS: {fps:5.1f} | last inference: {self._last_inf_ms:6.1f} ms "
                f"| dets in last frame: {len(out.detections)}"
            )
            self._n_frames = 0
            self._t_window_start = now




def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
