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

        self._term_thread = Event()
        self.record = False
        self.burst_duration = -1
        self.burst_start = -1
        self.burst_end = -1

    def burst_callback(self, msg: BurstRecordCommand):
        self._term_thread.clear()
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

        self.skip_counter += 1
        if self.skip_counter < self.record_every_nth_frame:
            return

        img = self.cv_bridge.imgmsg_to_cv2(msg)
        frame_id = int(msg.header.frame_id)
        timestamp = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        if timestamp < self.burst_start or timestamp > self.burst_end:
            self.record = False
            if not self.save_as_single_video:
                self._term_thread.set()
                self.initialized = False
            return

        if not self.initialized:
            self.output_shape = (img.shape[1], img.shape[0])
            self.is_color = (len(img.shape) == 3)
            if self.save_as_single_video:
                self.filename = f'{self.output_filename}'
            else:
                self.filename = f'{self.output_filename}_{datetime.now().strftime("%y%m%d_%H%M%S")}'
            self.spawn_thread(self.grab_and_save_frame, daemon=True)
            self.initialized = True

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
            if '-rc' not in self.encoder_args:
                cmd.extend(['-rc', 'constqp', '-qp', '18'])
        cmd.extend(self.encoder_args)
        cmd.append(f'{self.filename}.mp4')
        pipe = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        stamps = open(self.filename + '_timestamps.csv', 'w')
        stamps.write('frame_id,timestamp\n')
        if self.verbose:
            self.print(
                f'Saving csv to: {self.filename}_timestamps.csv\n'
                f'Saving csv to: {self.filename}.mp4\n'
                f'Pipes open; ready to fetch from buffer!\n'
            )

        while not self._term_thread.is_set():
            img, frame_id, timestamp = self.buffer.get(block=True)
            if img is None:
                break
            stamps.write(f'{frame_id},{timestamp}\n')
            pipe.stdin.write(img.astype(np.uint8).tobytes())
            pipe.stdin.flush()

        self.buffer.queue.clear()
        self.buffer.all_tasks_done.notify_all()
        self.buffer.unfinished_tasks = 0
        pipe.stdin.close()
        pipe.wait()
        stamps.close()
        return

    def on_destroy(self):
        self._term_thread.set()
        self.record = False
        self.buffer.put((None, None, None))
        with self.buffer.mutex:
            self.buffer.queue.clear()
            self.buffer.all_tasks_done.notify_all()
            self.buffer.unfinished_tasks = 0
        self.buffer.join()
        if self.verbose:
            self.print('All pipes and buffers joined and released cleanly.')


def main():
    rclpy.init()
    BurstVideoSaver().run()


if __name__ == '__main__':
    main()
