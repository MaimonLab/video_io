import cv2
from cv_bridge import CvBridge
from datetime import datetime
import numpy as np
from queue import Queue
import rclpy
from sensor_msgs.msg import Image
import subprocess
from threading import Event

from video_io.video_saver import VideoSaver
from video_io_cpp.msg import BurstRecordCommand


class BurstVideoSaver(VideoSaver):
    def __init__(self):
        super().__init__()

        self.default_param = {
            'image_topic': 'camera/image',
            'burst_record_command_topic': 'experiment_logic/burst_record_command',
            'output_fps': 60.0,
            'codec': 'mjpeg',
            'quality': 24,
            'encoder_args': [],
            'record_every_nth_frame': 1,
            'burn_timestamp': False,
            'save_as_single_video': True,
            'verbose': False,
            'output_filename': ''
        }

        self.register_subscriber(
            BurstRecordCommand, self.burst_record_command_topic,
            self.burst_callback,
            reliable=False,
            queue_size=10,
            log=True,
        )

        self.output_filestem = self.output_filename
        self.record = False
        self.burst_duration = -1
        self.burst_start = -1
        self.burst_end = -1

    def burst_callback(self, msg: BurstRecordCommand):
        self.term_ev.clear()
        self.record = True
        self.burst_duration = msg.record_duration_s
        self.burst_start = msg.header.stamp.nanosec + 1e9 * msg.header.stamp.sec
        self.burst_end = self.burst_start + 1e9 * self.burst_duration
        self.log_msg(self.log_burst_record_command, msg)
        if self.verbose:
            self.print(
                f'\nBurst received for {self.burst_duration}\n'
                f'\tStart: {self.burst_start}\n'
                f'\tEnd: {self.burst_end}\n\n'
            )
        return

    def image_callback(self, msg: Image):
        if not self.record:
            return

        timestamp = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        if timestamp < self.burst_start or timestamp > self.burst_end:
            self.record = False
            if not self.save_as_single_video:
                self.is_init = False
                self.term_ev.set()
            return

        if not self.is_init:
            self.buffer = Queue(50)
            if self.save_as_single_video:
                self.output_filename = f'{self.output_filestem}'
            else:
                self.output_filename = f'{self.output_filestem}_{datetime.now().strftime("%y%m%d_%H%M%S")}'

        super().image_callback(msg)
        return


def main():
    rclpy.init()
    BurstVideoSaver().run()


if __name__ == '__main__':
    main()
