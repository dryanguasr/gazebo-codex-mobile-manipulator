#!/usr/bin/env python3
"""Run the required simulator-level negative pick-and-place scenarios."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def residue_for(process_group, seed, world):
    completed = subprocess.run(
        ['ps', '-eo', 'pid=,pgid=,args='],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    residue = []
    seed_token = f'--seed {seed} '
    for line in completed.stdout.splitlines():
        fields = line.strip().split(maxsplit=2)
        if len(fields) < 3:
            continue
        same_group = fields[1].isdigit() and int(fields[1]) == process_group
        same_gazebo = (
            'gz sim ' in fields[2]
            and seed_token in fields[2]
            and str(world) in fields[2]
        )
        if same_group or same_gazebo:
            residue.append(line.strip())
    return residue


def terminate_group(process):
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def scenario_definitions(repo):
    large = (
        repo
        / 'src/mobile_manipulator/worlds/pick_object_large.sdf'
    )
    return [
        {
            'name': 'object_absent',
            'args': ['spawn_object:=false'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'object_024m_incompatible',
            'args': [f'object_sdf:={large}'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'proximity_without_valid_contacts',
            'args': ['object_y:=-0.175'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'invalid_tf_frame',
            'args': ['gate_grasp_frame:=missing_grasp_frame'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'controller_unavailable',
            'args': ['action_name:=/missing_controller/follow_joint_trajectory'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'base_in_motion',
            'args': ['negative_scenario:=base_motion'],
            'terminal': 'FAILED',
            'max_attach': 0,
        },
        {
            'name': 'cancel_during_transport',
            'args': ['negative_scenario:=cancel_during_transport'],
            'terminal': 'CANCELLED',
            'max_attach': 1,
        },
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root')
    parser.add_argument('--timeout-s', type=float, default=180.0)
    parser.add_argument('--seed-start', type=int, default=301)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    root = (
        Path(args.output_root).resolve()
        if args.output_root
        else repo / 'results' / 'verified' / f'pick_A1_negative_{timestamp}'
    )
    if root.exists():
        raise SystemExit(f'Refusing to overwrite existing suite: {root}')
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
    gz_versions = subprocess.run(
        ['gz', 'sim', '--versions'],
        cwd=repo,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    ).stdout.strip()
    versions = {
        'gz': gz_versions,
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
    }
    world = (
        repo
        / 'install/mobile_manipulator/share/mobile_manipulator/worlds'
        / 'pick_and_place.sdf'
    )
    reports = []
    for index, scenario in enumerate(scenario_definitions(repo)):
        seed = args.seed_start + index
        run_dir = root / scenario['name']
        run_dir.mkdir()
        command = [
            'ros2', 'launch', 'mobile_manipulator',
            'pick_and_place.launch.py',
            f'output_dir:={run_dir}',
            f'run_id:=negative_{scenario["name"]}',
            f'source_sha:={sha}',
            f'source_dirty:={str(dirty).lower()}',
            f'seed:={seed}',
            'capture_evidence:=false',
            *scenario['args'],
        ]
        timed_out = False
        with (run_dir / 'launch.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen(
                command,
                cwd=repo,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            try:
                returncode = process.wait(timeout=args.timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                terminate_group(process)
                returncode = process.returncode
        time.sleep(1.0)
        residue = residue_for(process.pid, seed, world)
        result_path = run_dir / 'run.json'
        result = (
            json.loads(result_path.read_text(encoding='utf-8'))
            if result_path.exists()
            else None
        )
        initialization_path = run_dir / 'initialization.json'
        initialization = (
            json.loads(initialization_path.read_text(encoding='utf-8'))
            if initialization_path.exists()
            else None
        )
        initialization_required = scenario['name'] != 'object_absent'
        initialization_valid = (
            not initialization_required
            or bool(
                initialization
                and initialization.get('status') == 'passed'
                and initialization.get('verified_free') is True
                and initialization.get('excluded_from_measured_run') is True
            )
        )
        no_false_success = bool(
            result
            and result.get('status') != 'passed'
            and result.get('terminal_state') == scenario['terminal']
            and int(result.get('attach_count', 0)) <= scenario['max_attach']
        )
        if scenario['name'] == 'cancel_during_transport' and result:
            authorizations = result.get('attach_authorizations', [])
            no_false_success = (
                no_false_success
                and len(authorizations) == 1
                and result.get('detach_count') == 1
            )
        passed = (
            returncode == 0
            and not timed_out
            and not residue
            and initialization_valid
            and no_false_success
        )
        report = {
            **scenario,
            'seed': seed,
            'source_sha': sha,
            'source_dirty': dirty,
            'config': str(config.relative_to(repo)),
            'config_sha256': config_hash,
            'software_versions': versions,
            'returncode': returncode,
            'timed_out': timed_out,
            'process_residue': residue,
            'run_json_present': result is not None,
            'initialization_required': initialization_required,
            'initialization_json_present': initialization is not None,
            'initialization_valid': initialization_valid,
            'initialization': initialization,
            'no_false_success': no_false_success,
            'passed': passed,
            'result': result,
        }
        reports.append(report)
        (run_dir / 'negative_result.json').write_text(
            json.dumps(report, indent=2) + '\n',
            encoding='utf-8',
        )
        print(
            f'[{index + 1}/{len(scenario_definitions(repo))}] '
            f'{scenario["name"]}: {"PASS" if passed else "FAIL"}',
            flush=True,
        )
    summary = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_sha': sha,
        'source_dirty': dirty,
        'config': str(config.relative_to(repo)),
        'config_sha256': config_hash,
        'software_versions': versions,
        'attempted': len(reports),
        'passed': sum(report['passed'] for report in reports),
        'failed': sum(not report['passed'] for report in reports),
        'accepted': all(report['passed'] for report in reports),
        'reports': reports,
    }
    (root / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n',
        encoding='utf-8',
    )
    print(
        f'negative_suite={root} accepted={summary["accepted"]}',
        flush=True,
    )
    return 0 if summary['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
