import rclpy
from rclpy.time import Time

from maimon_classes.basic_node import BasicNode
from video_io_cpp.msg import BurstRecordCommand


class BurstRecordCommandPublisher(BasicNode):
    def __init__(self):
        super().__init__()

        self.default_param = {
            'burst_record_command_topic': 'experiment_logic/burst_record_command',
            'burst_interval_s': 10.0,
            'record_duration_s': 1.0,
        }

        self.pub_burst_record_command = self.register_publisher(
            BurstRecordCommand, self.burst_record_command_topic,
            reliable=False, queue_size=10,
        )

        self.timer = self.create_timer(self.burst_interval_s, self.timer_callback)

    def timer_callback(self):
        msg = BurstRecordCommand()
        timestamp = self.get_clock().now().nanoseconds
        msg.header.stamp = Time(nanoseconds=timestamp).to_msg()
        msg.record_duration_s = self.record_duration_s
        self.pub_burst_record_command.publish(msg)
        return


def main():
    rclpy.init()
    BurstRecordCommandPublisher().run()


if __name__ == '__main__':
    main()
