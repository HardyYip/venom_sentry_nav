"""Wait briefly for the original obstacle scan transform in the ROS executor."""

from collections import deque
import time

from rclpy.time import Time
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


class SourceTimeObstacleQueue:
    def __init__(self, source, target, clock, buffer, publish, drop, counts):
        self.source = source
        self.target = target
        self.clock = clock
        self.buffer = buffer
        self.publish = publish
        self.drop = drop
        self.counts = counts
        self.pending = deque()
        self.latest_source = -1.

    def receive(self, msg):
        self.counts['received'] += 1
        stamp = msg.header.stamp
        source_time = stamp.sec + stamp.nanosec * 1e-9
        ros = self.clock.now().nanoseconds * 1e-9
        if (msg.header.frame_id != self.source or
                not (0 < source_time and source_time > self.latest_source and
                     -.1 <= ros - source_time <= .25)):
            self.drop('wrong frame, nonmonotonic or stale source')
            return
        self.latest_source = source_time
        if len(self.pending) >= 8:
            self.pending.popleft()
            self.drop('pending source queue full')
        self.pending.append((msg, source_time, time.monotonic(), False))
        self.drain()

    def drain(self):
        while self.pending:
            msg, source_time, received, waited = self.pending[0]
            ros = self.clock.now().nanoseconds * 1e-9
            if ros - source_time > .25 or time.monotonic() - received > .25:
                self.pending.popleft()
                self.drop('source-time TF unavailable within original freshness window')
                continue
            try:
                transform = self.buffer.lookup_transform(
                    self.target, self.source, Time.from_msg(msg.header.stamp)).transform
            except (ConnectivityException, ExtrapolationException, LookupException):
                if not waited:
                    self.pending[0] = (msg, source_time, received, True)
                    self.counts['waiting_for_source_tf'] += 1
                return
            self.pending.popleft()
            self.publish(msg, transform, waited)
