"""Check the same-epoch retry budget and fail-closed guards."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from core import Recovery
from supervisor import Supervisor


def check():
    with TemporaryDirectory() as temp:
        log = Path(temp) / 'launch.log'
        log.write_text('IMU_INPUT_FAULT_LATCH previous=5.0 current=5.15 gap=0.15')
        core = Recovery(100.0)
        core.epoch = 1
        core.restart_stamp = 33.0
        process = SimpleNamespace(
            direct_goal_preparation=True, startup_retries={1: 2}, core=core,
            odom_at=100.0, odom_stamp=30.0, odom_progress=100.0, startup_retry_check_at=100.0, ros=lambda: 25.1,
            independent_stationary=lambda *args, **kwargs: True,
            startup_log=log, startup_began_at=100.0, write=lambda *args, **kwargs: None)
        process.odometry_healthy = lambda when: Supervisor.odometry_healthy(process, when)
        retry = Supervisor.retry_failed_startup
        assert not retry(process, 110.0, True, True, False)
        assert core.state == 'VERIFY' and not core.permit
        core.tick(131.0)
        assert core.state == 'FAILED' and not core.permit

        process.core = Recovery(200.0)
        process.startup_retries.clear()
        process.odom_stamp, process.odom_at = 0.0, -1e20
        process.startup_began_at = 200.0
        assert not retry(process, 210.0, False, True, False)
        assert not retry(process, 210.0, True, False, False)
        process.independent_stationary = lambda *args, **kwargs: False
        assert not retry(process, 210.0, True, True, False)
        process.independent_stationary = lambda *args, **kwargs: True
        process.odom_stamp, process.odom_at, process.odom_progress = 25.0, 214.0, 214.0
        assert not retry(process, 214.0, True, True, False)
        assert not process.core.permit and not process.startup_retries
        process.odom_stamp, process.odom_at = 0.0, -1e20
        log.write_text('failed to send response to /ns/container/_container/load_node '
                       '(timeout): client will not receive response')
        assert not retry(process, 216.0, True, True, True)
        assert retry(process, 218.0, True, True, False)
        assert not process.core.permit
        process.core.restarted(220.0, 30.0, True)
        assert not retry(process, 228.0, True, True, False)
        assert process.startup_retries == {0: 1}

        process.core = Recovery(250.0)
        process.core.epoch = 1
        process.core.restart_stamp = 23.0
        process.odom_stamp, process.odom_at, process.odom_progress = 25.0, 256.0, 256.0
        log.write_text('IMU_INPUT_FAULT_LATCH previous=25.05 current=25.165 gap=0.115')
        emitted = []
        process.write = lambda kind, **event: emitted.append((kind, event))
        assert not retry(process, 256.05, True, True, False)
        assert retry(process, 257.0, True, True, False)
        assert not process.core.permit
        assert emitted[0][1]['trigger'] == 'point_lio_imu_input_fault_during_verification'
    print('PASS bounded retries, stationary/terminal/odometry guards, repeated component timeout held')


if __name__ == '__main__':
    check()
