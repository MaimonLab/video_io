import queue

import cv2
from cv_bridge import CvBridge
from datetime import datetime
import numpy as np
from queue import Queue
import rclpy
from sensor_msgs.msg import Image
import subprocess
from threading import Event

from maimon_classes.basic_node import BasicNode


class VideoSaver(BasicNode):
    def __init__(self):
        super().__init__(
            automatically_declare_parameters_from_overrides=True
        )

        self.default_param = {
            'image_topic': 'camera/image',
            'output_fps': 60.0,
            'codec': 'mjpeg',
            'quality': 24,
            'encoder_args': [],
            'record_every_nth_frame': 1,
            'burn_timestamp': False,
            'quit_after_s_seconds': -1,
            'verbose': False,
            'output_filename': ''
        }

        self.output_fps = getattr(self, 'output_fps_double', self.output_fps)
        self.verbose = getattr(self, 'verbose_logging', self.verbose)

        self.cv_bridge = CvBridge()
        self.is_init = False
        self.skip_counter = 0

        self.buffer = Queue(50)
        self.pipe = self.stamps = None
        self.term_ev = Event()
        self.term_ev.clear()

        self.register_subscriber(
            Image, self.image_topic,
            self.image_callback,
            reliable=False,
            queue_size=50
        )

        if self.quit_after_s_seconds > 0:
            self.print_warning(f'Created timer that will close after '
                               f'{self.quit_after_s_seconds} seconds!')
            self.timer = self.create_timer(self.quit_after_s_seconds, self.quit_node)

    def image_callback(self, msg: Image):
        self.skip_counter += 1
        if self.skip_counter < self.record_every_nth_frame:
            return

        img = self.cv_bridge.imgmsg_to_cv2(msg)
        frame_id = int(msg.header.frame_id)
        timestamp = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)

        if not self.is_init:
            self.output_shape = (img.shape[1], img.shape[0])
            self.is_color = (len(img.shape) == 3)
            self.is_init = True
            self.spawn_thread(self.grab_and_save_frame, daemon=True)

        self.add_timestamp(img, frame_id, timestamp)
        self.buffer.put((img, frame_id, timestamp))
        self.skip_counter = 0
        return

    def grab_and_save_frame(self):
        cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{self.output_shape[0]}x{self.output_shape[1]}',
            '-r', f'{self.output_fps}',
            '-pix_fmt', 'rgb24' if self.is_color else 'gray',
            '-i', '-', '-an',
            '-vcodec', self.codec,
        ]
        if 'nvenc' in self.codec:
            cmd = cmd[:2] + ['-hwaccel', 'cuda', '-hwaccel_output_format', 'cuda'] + cmd[2:]
        if '264' in self.codec or '265' in self.codec or 'evc' in self.codec:
            cmd.extend(['-qp', f'{int(self.quality)}'])
        cmd.extend(self.encoder_args)
        cmd.append(f'{self.output_filename}.mp4')
        self.pipe = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

        self.stamps = open(self.output_filename + '_timestamps.csv', 'w')
        self.stamps.write('frame_id,timestamp\n')

        if self.verbose:
            self.print(
                f'\nSaving csv to: {self.output_filename}_timestamps.csv\n'
                f'Saving csv to: {self.output_filename}.mp4\n'
                f'Pipes open; ready to fetch from buffer!'
            )

        while not self.term_ev.is_set():
            try:
                img, frame_id, timestamp = self.buffer.get(timeout=0.5)
            except queue.Empty:
                continue
            self.stamps.write(f'{frame_id},{timestamp}\n')
            self.pipe.stdin.write(img.astype(np.uint8).tobytes())
            self.pipe.stdin.flush()

        # shutting down async objects
        with self.buffer.mutex:
            self.buffer.queue.clear()
            self.buffer.all_tasks_done.notify_all()
            self.buffer.unfinished_tasks = 0
        self.buffer.join()
        self.stamps.close()
        self.pipe.stdin.close()
        self.pipe.wait()
        if self.verbose:
            self.print_warning('All pipes and buffers joined and released cleanly.')
        return

    def add_timestamp(self, img, frame_id, timestamp):
        if not self.burn_timestamp:
            return img

        timestamp = datetime.fromtimestamp(timestamp / 1e9).strftime("%y/%m/%d %H:%M:%S")
        cv2.rectangle(img, (0, self.output_shape[1] - 17), (165, self.output_shape[1]), 0, -1)
        cv2.putText(
            img, timestamp,
            (0, self.output_shape[1] - 5), cv2.FONT_HERSHEY_PLAIN,
            1, (255, 255, 255), 1,
        )
        cv2.rectangle(img, (0, self.output_shape[1] - 34), (66, self.output_shape[1] - 17), 0, -1)
        cv2.putText(
            img, f"id: {frame_id}",
            (0, self.output_shape[1] - 22), cv2.FONT_HERSHEY_PLAIN,
            1, (255, 255, 255), 1,
        )
        return img

    def quit_node(self):
        self.print_warning('Closing video saver node...')
        self.term_ev.set()
        # self.buffer.put((None, None, None))
        raise KeyboardInterrupt

    def on_destroy(self):
        if not self.is_init or self.pipe is None or self.stamps is None:
            self.print_warning(
                f'VideoSaver never initialized! '
                f'Pipe may have never received a frame to save or '
                f'the node process may have shutdown prematurely.')
            return

        self.term_ev.set()


def main():
    rclpy.init()
    VideoSaver().run()


if __name__ == '__main__':
    main()
