#!/usr/bin/env python3
"""Run an immutable A1 campaign and aggregate independent run reports."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def run_text(command, cwd):
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return {
        'command': command,
        'returncode': completed.returncode,
        'output': completed.stdout.strip(),
    }


def process_residue(process_group, seed, world):
    completed = subprocess.run(
        ['ps', '-eo', 'pid=,pgid=,args='],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    residue = []
    for line in completed.stdout.splitlines():
        fields = line.strip().split(maxsplit=2)
        if len(fields) < 3:
            continue
        same_group = fields[1].isdigit() and int(fields[1]) == process_group
        same_gazebo = (
            'gz sim ' in fields[2]
            and f'--seed {seed} ' in fields[2]
            and str(world) in fields[2]
        )
        if same_group or same_gazebo:
            residue.append(line.strip())
    return residue


def stop_timed_out_process(process):
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
        return 'SIGINT'
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)
        return 'SIGTERM'


def load_json(path):
    with path.open(encoding='utf-8') as handle:
        return json.load(handle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--seed-start', type=int, default=101)
    parser.add_argument('--timeout-s', type=float, default=180.0)
    parser.add_argument('--capture-index', type=int, default=1)
    parser.add_argument('--output-root')
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')

    repo = Path(__file__).resolve().parents[1]
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    root = (
        Path(args.output_root).resolve()
        if args.output_root
        else repo / 'results' / 'verified' / f'pick_A1_{timestamp}'
    )
    if root.exists():
        raise SystemExit(f'Refusing to overwrite existing campaign: {root}')
    root.mkdir(parents=True)

    sha = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=repo, text=True
    ).strip()
    dirty = bool(
        subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=repo, text=True
        ).strip()
    )
    config = repo / 'src/mobile_manipulator/config/pick_place_a1.yaml'
    config_hash = hashlib.sha256(config.read_bytes()).hexdigest()
    provenance = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_sha': sha,
        'source_dirty': dirty,
        'config': str(config.relative_to(repo)),
        'config_sha256': config_hash,
        'world': 'pick_and_place.sdf',
        'launch': 'pick_and_place.launch.py',
        'requested_runs': args.runs,
        'seed_start': args.seed_start,
        'timeout_s': args.timeout_s,
        'versions': {
            'gz': run_text(['gz', 'sim', '--versions'], repo),
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
            'package_prefix': run_text(
                ['ros2', 'pkg', 'prefix', 'mobile_manipulator'], repo
            ),
        },
    }
    (root / 'provenance.json').write_text(
        json.dumps(provenance, indent=2) + '\n',
        encoding='utf-8',
    )

    attempts = []
    world = (
        repo
        / 'install/mobile_manipulator/share/mobile_manipulator/worlds'
        / 'pick_and_place.sdf'
    )
    for index in range(1, args.runs + 1):
        run_id = f'a1_{index:02d}'
        seed = args.seed_start + index - 1
        run_dir = root / run_id
        run_dir.mkdir()
        capture = index == args.capture_index
        command = [
            'ros2', 'launch', 'mobile_manipulator',
            'pick_and_place.launch.py',
            f'output_dir:={run_dir}',
            f'run_id:={run_id}',
            f'source_sha:={sha}',
            f'source_dirty:={str(dirty).lower()}',
            f'seed:={seed}',
            f'capture_evidence:={str(capture).lower()}',
        ]
        started = datetime.now(timezone.utc)
        timed_out = False
        termination = None
        log_path = run_dir / 'launch.log'
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(
                command,
                cwd=repo,
                text=True,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                returncode = process.wait(timeout=args.timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                termination = stop_timed_out_process(process)
                returncode = process.returncode
        time.sleep(1.0)
        residue = process_residue(process.pid, seed, world)
        result_path = run_dir / 'run.json'
        result = load_json(result_path) if result_path.exists() else None
        initialization_path = run_dir / 'initialization.json'
        initialization = (
            load_json(initialization_path)
            if initialization_path.exists()
            else None
        )
        initialization_valid = bool(
            initialization
            and initialization.get('status') == 'passed'
            and initialization.get('verified_free') is True
            and initialization.get('excluded_from_measured_run') is True
        )
        if (
            timed_out
            or residue
            or result is None
            or not initialization_valid
        ):
            classification = 'invalid'
        elif result.get('status') == 'passed':
            classification = 'success'
        else:
            classification = 'failure'
        attempt = {
            'attempt': index,
            'run_id': run_id,
            'seed': seed,
            'source_sha': sha,
            'source_dirty': dirty,
            'config': str(config.relative_to(repo)),
            'config_sha256': config_hash,
            'provenance_file': '../provenance.json',
            'software_versions': provenance['versions'],
            'capture_evidence': capture,
            'started_utc': started.isoformat(),
            'completed_utc': datetime.now(timezone.utc).isoformat(),
            'returncode': returncode,
            'timed_out': timed_out,
            'timeout_termination': termination,
            'process_group': process.pid,
            'process_residue': residue,
            'log_present': log_path.exists(),
            'run_json_present': result is not None,
            'initialization_json_present': initialization is not None,
            'initialization_valid': initialization_valid,
            'initialization': initialization,
            'classification': classification,
            'result': result,
        }
        attempts.append(attempt)
        (run_dir / 'attempt.json').write_text(
            json.dumps(attempt, indent=2) + '\n',
            encoding='utf-8',
        )
        print(
            f'[{index}/{args.runs}] {run_id} seed={seed} '
            f'class={classification} rc={returncode}',
            flush=True,
        )

    successes = sum(
        item['classification'] == 'success' for item in attempts
    )
    failures = sum(
        item['classification'] == 'failure' for item in attempts
    )
    invalid = sum(
        item['classification'] == 'invalid' for item in attempts
    )
    grasps = sum(
        bool(
            item['result']
            and item['result'].get('criteria', {}).get('grasp_contract_verified')
            and item['result'].get('criteria', {}).get('lift_at_least_50mm')
        )
        for item in attempts
    )
    placements = sum(
        bool(
            item['result']
            and item['result'].get('criteria', {}).get('placement_within_30mm')
            and item['result'].get('criteria', {}).get('stable_at_least_2s')
            and item['result'].get('criteria', {}).get('released')
        )
        for item in attempts
    )
    no_crash_or_residue = all(
        item['returncode'] == 0
        and not item['timed_out']
        and not item['process_residue']
        for item in attempts
    )
    acceptance = {
        'exactly_10_attempts': args.runs == 10,
        '10_of_10_without_crash_or_residue': (
            args.runs == 10 and no_crash_or_residue and invalid == 0
        ),
        'at_least_9_of_10_grasps': args.runs == 10 and grasps >= 9,
        'at_least_9_of_10_placements': args.runs == 10 and placements >= 9,
        'all_run_logs_present': all(
            item['log_present'] and item['run_json_present']
            for item in attempts
        ),
        'all_initializations_verified': all(
            item['initialization_valid'] for item in attempts
        ),
        'clean_source': not dirty,
    }
    summary = {
        **provenance,
        'attempted': args.runs,
        'valid': args.runs - invalid,
        'success': successes,
        'failure': failures,
        'invalid': invalid,
        'grasp_successes': grasps,
        'placement_successes': placements,
        'no_crash_or_process_residue': no_crash_or_residue,
        'acceptance': acceptance,
        'accepted': all(acceptance.values()),
        'attempts': attempts,
    }
    (root / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n',
        encoding='utf-8',
    )
    print(
        f'campaign={root} success={successes}/{args.runs} '
        f'invalid={invalid} accepted={summary["accepted"]}',
        flush=True,
    )
    return 0 if summary['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
