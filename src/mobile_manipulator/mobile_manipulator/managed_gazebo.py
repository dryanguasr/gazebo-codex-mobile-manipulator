"""Run one Gazebo server in an isolated, deterministically reaped group."""

import argparse
import os
import signal
import subprocess
import time


SHUTDOWN_REQUESTED = False


def request_shutdown(_signum, _frame):
    global SHUTDOWN_REQUESTED
    SHUTDOWN_REQUESTED = True


def wait_for_exit(process, timeout_s):
    deadline = time.monotonic() + timeout_s
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    return process.poll() is not None


def reap_process_group(process):
    if process.poll() is not None:
        return 'already_exited'
    for action, sig, timeout_s in (
        ('SIGINT', signal.SIGINT, 1.5),
        ('SIGTERM', signal.SIGTERM, 1.5),
        ('SIGKILL', signal.SIGKILL, 1.0),
    ):
        os.killpg(process.pid, sig)
        if wait_for_exit(process, timeout_s):
            return action
    return 'unreaped'


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('world')
    parsed = parser.parse_args(args)
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    command = [
        'gz', 'sim', '-r', '-s', '--seed', str(parsed.seed),
        parsed.world, '--force-version', '8',
    ]
    environment = os.environ.copy()
    environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join([
        environment.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        environment.get('LD_LIBRARY_PATH', ''),
    ])
    process = subprocess.Popen(
        command, env=environment, start_new_session=True
    )
    print(
        f'managed_gazebo child_pid={process.pid} seed={parsed.seed} '
        f'world={parsed.world}',
        flush=True,
    )
    try:
        while process.poll() is None and not SHUTDOWN_REQUESTED:
            time.sleep(0.1)
        if SHUTDOWN_REQUESTED:
            action = reap_process_group(process)
            print(
                f'managed_gazebo shutdown child_pid={process.pid} '
                f'action={action}',
                flush=True,
            )
            return 0 if action != 'unreaped' else 1
        return int(process.returncode or 0)
    finally:
        if process.poll() is None:
            reap_process_group(process)


if __name__ == '__main__':
    raise SystemExit(main())
