"""Exercise the installed lifecycle smoother against a slow, paused and reset ROS clock."""
import json
from math import hypot
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rosgraph_msgs.msg import Clock


def check(output):
    rclpy.init()
    node = rclpy.create_node('smoother_clock_probe', namespace='/rmul_smoother_clock_probe')
    clock = node.create_publisher(Clock, '/clock', 10)
    cmd = node.create_publisher(Twist, 'cmd_vel', 10)
    state = node.create_client(ChangeState, 'velocity_smoother/change_state')
    samples = []
    planar_speeds = []
    sim_ns = 2_000_000_000
    target = Twist()
    target.linear.y = 2.0
    phases = {}

    def observe(message):
        samples.append((sim_ns / 1e9, message.linear.y))
        planar_speeds.append(hypot(message.linear.x, message.linear.y))

    node.create_subscription(Twist, 'cmd_vel_smoothed', observe, 10)

    def step(duration, rate=.5, command=False):
        nonlocal sim_ns
        deadline = time.monotonic() + duration
        previous = time.monotonic_ns()
        while time.monotonic() < deadline:
            now = time.monotonic_ns()
            sim_ns += int((now - previous) * rate)
            previous = now
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(sim_ns, 1_000_000_000)
            clock.publish(msg)
            if command:
                cmd.publish(target)
            rclpy.spin_once(node, timeout_sec=.004)
            time.sleep(.003)

    def transition(which):
        request = ChangeState.Request()
        request.transition.id = which
        future = state.call_async(request)
        deadline = time.monotonic() + 4
        while not future.done() and time.monotonic() < deadline:
            step(.03)
        assert future.done() and future.result().success, which

    try:
        assert state.wait_for_service(timeout_sec=8)
        step(.2)
        transition(Transition.TRANSITION_CONFIGURE)
        transition(Transition.TRANSITION_ACTIVATE)
        step(.3)
        start_at = sim_ns / 1e9
        step(1.2, command=True)
        assert len(samples) >= 12
        peak = max(v for _, v in samples)
        elapsed = sim_ns / 1e9 - start_at
        # A 50 ms wall-timer sibling ignoring /clock would reach 2 m/s here.
        assert .83 < peak < 1.12, (elapsed, peak)
        assert abs(peak - 1.75 * elapsed) < .08, (elapsed, peak)
        phases['slow_clock'] = dict(elapsed_sim_s=elapsed, peak_mps=peak, sample_count=len(samples))
        # Let the wall timer publish its final update for the last advancing clock stamp.
        step(.15, rate=0.0, command=True)
        paused_velocity = samples[-1][1]
        start_pause = len(samples)
        step(.35, rate=0.0, command=True)
        assert len(samples) > start_pause
        pause_max = max(abs(value - paused_velocity) for _, value in samples[start_pause:])
        assert pause_max < .0001, pause_max
        phases['pause'] = dict(max_delta_mps=pause_max, wall_seconds=.35)
        start_reset = len(samples)
        sim_ns -= 1_000_000_000
        step(.25, command=False)
        after_reset = samples[start_reset:]
        # A command published before the smoother received the new clock can
        # still be in flight. Verify that the zero output arrives and persists.
        zero_index = next((i for i, (_, value) in enumerate(after_reset) if abs(value) < .0001), None)
        assert zero_index is not None, after_reset
        after_zero = after_reset[zero_index:]
        assert all(abs(value) < .0001 for _, value in after_zero), after_reset
        phases['rollback'] = dict(samples_after=len(after_zero), max_mps=max(v for _, v in after_zero))

        target.linear.x = 2.0
        start_diagonal = len(planar_speeds)
        step(2.2, rate=1.0, command=True)
        peak_diagonal = max(planar_speeds[start_diagonal:])
        assert 1.95 <= peak_diagonal <= 2.00001, peak_diagonal
        phases['diagonal'] = dict(peak_mps=peak_diagonal)
        transition(Transition.TRANSITION_DEACTIVATE)
        output.write_text(json.dumps(phases, indent=2) + '\n')
        print(json.dumps(phases), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    check(Path(sys.argv[1]))
