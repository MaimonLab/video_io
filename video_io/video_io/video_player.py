import time

import cv2
from cv_bridge import CvBridge
from datetime import datetime
from queue import Queue
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image

from maimon_classes.basic_node import BasicNode


class VideoPlayer(BasicNode):
    def __init__(self):
        super().__init__(
            automatically_declare_parameters_from_overrides=True
        )

        self.default_param = {
            'image_topic': 'camera/image',
            'filename': '',
            'loop_play': False,
            'publish_as_color': True,
            'publish_frequency': -1,
            'publish_every_nth_frame': 1,
            'start_frame': 0,
            'downsample_ratio': 1.0,
            'burn_timestamp': False,
            'reliable': False,
            'verbose': False,
        }

        # backwards compatibility with some inconsistent parameter names
        self.publish_frequency = getattr(self, 'publish_frequency_double', self.publish_frequency)
        self.burn_timestamp = getattr(self, 'add_timestamp', self.burn_timestamp)
        self.reliable = getattr(self, 'qos_image_publish_reliable', self.reliable)
        self.verbose = getattr(self, 'verbose_logging', self.verbose)

        self.cap = cv2.VideoCapture(self.filename)
        if not self.cap.isOpened():
            self.print_error("Could not open file!")
            raise IOError

        self.frame_count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.publish_frequency < 0:
            self.publish_frequency = self.fps

        if self.verbose:
            self.print(
                f'Playing video from: {self.filename}\n'
                f'format: h:{self.h} |  w:{self.w} | '
                f'fps:{self.publish_frequency} | total_frames:{self.frame_count}\n'
            )

        self.bridge = CvBridge()
        self.pub_video_player = self.register_publisher(
            Image, self.image_topic, reliable=self.reliable
        )

        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame)
        self.counter = 0

        self.create_timer(1/self.publish_frequency, self.play_next_frame)

    def play_next_frame(self):
        for n in range(self.publish_every_nth_frame):
            ret, frame = self.cap.read()
        if not ret:
            if not self.loop_play:
                self.print('Finished playback. Exiting.')
                raise KeyboardInterrupt
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

        if self.publish_as_color and frame.shape[-1] == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if not self.publish_as_color and frame.shape[-1] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        timestamp = self.get_clock().now().nanoseconds
        frame = self._add_timestamp(frame, str(self.counter), timestamp)
        img_msg = self.bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = str(self.counter)
        img_msg.header.stamp = Time(nanoseconds=timestamp).to_msg()
        self.pub_video_player.publish(img_msg)

        self.counter += 1

    def _add_timestamp(self, img, frame_id, timestamp):
        if not self.burn_timestamp:
            return img

        timestamp = datetime.fromtimestamp(timestamp / 1e9).strftime("%y/%m/%d %H:%M:%S")
        cv2.rectangle(img, (0, self.h - 17), (165, self.h), 0, -1)
        cv2.putText(
            img, timestamp,
            (0, self.h - 5), cv2.FONT_HERSHEY_PLAIN,
            1, (255, 255, 255), 1,
        )
        cv2.rectangle(img, (0, self.h - 34), (66, self.h - 17), 0, -1)
        cv2.putText(
            img, f"id: {frame_id}",
            (0, self.h - 22), cv2.FONT_HERSHEY_PLAIN,
            1, (255, 255, 255), 1,
        )
        return img


def main():
    rclpy.init()
    VideoPlayer().run()


if __name__ == '__main__':
    main()

