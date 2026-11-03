#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import time
from collections import deque

import rospy
from sensor_msgs.msg import Imu, Image
from livox_ros_driver2.msg import CustomMsg


def stats(values):
    if not values:
        return None
    return {
        'count': len(values),
        'mean': statistics.mean(values),
        'std': statistics.pstdev(values) if len(values) > 1 else 0.0,
        'min': min(values),
        'max': max(values),
    }


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    idx = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (idx - lo)


class Validator:
    def __init__(self, duration):
        self.duration = duration
        self.lidar_frames = []
        self.camera_frames = []
        self.imu_times = []
        self.lidar_backwards = 0
        self.lidar_duplicates = 0
        self.lidar_jumps = 0
        self.imu_backwards = 0
        self.imu_duplicates = 0
        self.camera_lidar_rows = []
        self.last_lidar_header = None
        self.last_imu_time = None

    def lidar_cb(self, msg):
        header = msg.header.stamp.to_sec()
        offsets = [p.offset_time for p in msg.points]
        offset_nonzero = sum(1 for x in offsets if x != 0)
        offset_dups = len(offsets) - len(set(offsets)) if offsets else 0
        offset_back = sum(1 for a, b in zip(offsets, offsets[1:]) if b < a)
        max_offset_ns = max(offsets) if offsets else 0
        begin = header
        end = header + max_offset_ns / 1e9
        point_count = len(offsets)
        if self.last_lidar_header is not None:
            dt = header - self.last_lidar_header
            if dt < 0:
                self.lidar_backwards += 1
            if abs(dt) < 1e-12:
                self.lidar_duplicates += 1
            if abs(dt - 0.1) > 0.02:
                self.lidar_jumps += 1
        self.last_lidar_header = header
        self.lidar_frames.append({
            'header': header,
            'begin': begin,
            'end': end,
            'duration_ms': (end - begin) * 1000.0,
            'point_count': point_count,
            'offset_nonzero': offset_nonzero,
            'offset_duplicates': offset_dups,
            'offset_backwards': offset_back,
            'offset_min_ns': min(offsets) if offsets else None,
            'offset_max_ns': max_offset_ns,
        })

    def imu_cb(self, msg):
        t = msg.header.stamp.to_sec()
        if self.last_imu_time is not None:
            if t < self.last_imu_time:
                self.imu_backwards += 1
            if abs(t - self.last_imu_time) < 1e-12:
                self.imu_duplicates += 1
        self.last_imu_time = t
        self.imu_times.append(t)

    def image_cb(self, msg):
        cam = msg.header.stamp.to_sec()
        self.camera_frames.append({'seq': msg.header.seq, 'time': cam})

    def finish(self):
        lidar_headers = [x['header'] for x in self.lidar_frames]
        lidar_periods_ms = [(b - a) * 1000.0 for a, b in zip(lidar_headers, lidar_headers[1:])]
        imu_periods_ms = [(b - a) * 1000.0 for a, b in zip(self.imu_times, self.imu_times[1:])]
        cam_times = [x['time'] for x in self.camera_frames]
        cam_periods_ms = [(b - a) * 1000.0 for a, b in zip(cam_times, cam_times[1:])]
        lidar_duration = (lidar_headers[-1] - lidar_headers[0]) if len(lidar_headers) > 1 else 0.0
        cam_duration = (cam_times[-1] - cam_times[0]) if len(cam_times) > 1 else 0.0
        imu_duration = (self.imu_times[-1] - self.imu_times[0]) if len(self.imu_times) > 1 else 0.0

        coverage = []
        for lf in self.lidar_frames:
            count = sum(1 for t in self.imu_times if lf['begin'] <= t <= lf['end'])
            first = next((t for t in self.imu_times if t >= lf['begin']), None)
            last = None
            for t in self.imu_times:
                if t <= lf['end']:
                    last = t
                elif t > lf['end']:
                    break
            coverage.append({
                'lidar_begin_time': lf['begin'],
                'lidar_end_time': lf['end'],
                'imu_first_time': first,
                'imu_last_time': last,
                'imu_sample_count': count,
                'covers_begin': first is not None and first - lf['begin'] <= 0.01,
                'covers_end': last is not None and lf['end'] - last <= 0.01,
            })

        point_diag = self.lidar_frames
        self.camera_lidar_rows = []
        sorted_lidar = sorted(self.lidar_frames, key=lambda x: x['begin'])
        lidar_idx = 0
        for cam_frame in self.camera_frames:
            cam = cam_frame['time']
            while lidar_idx + 1 < len(sorted_lidar) and sorted_lidar[lidar_idx]['end'] < cam:
                lidar_idx += 1
            lf = sorted_lidar[lidar_idx] if sorted_lidar else None
            relation = 'contains'
            if lf is None:
                continue
            if not (lf['begin'] <= cam <= lf['end']):
                candidates = [lf]
                if lidar_idx + 1 < len(sorted_lidar):
                    candidates.append(sorted_lidar[lidar_idx + 1])
                if lidar_idx > 0:
                    candidates.append(sorted_lidar[lidar_idx - 1])
                lf = min(candidates, key=lambda x: min(abs(cam - x['begin']), abs(cam - x['end'])))
                relation = 'nearest'
            self.camera_lidar_rows.append({
                'camera_seq': cam_frame['seq'],
                'camera_time': cam,
                'lidar_header_time': lf['header'],
                'lidar_frame_begin_time': lf['begin'],
                'lidar_frame_end_time': lf['end'],
                'camera_minus_lidar_begin_ms': (cam - lf['begin']) * 1000.0,
                'camera_minus_lidar_end_ms': (cam - lf['end']) * 1000.0,
                'match_type': relation,
            })

        cut_segments = []
        total_points = 0
        duplicate_interval_points = 0
        dropped_interval_points = 0
        for prev_cam, cur_cam in zip(cam_times, cam_times[1:]):
            pts = 0
            for lf in self.lidar_frames:
                # Count by frame ranges rather than every point for speed. This is a conservative
                # interval check of FAST-LIVO's intended rule, not a replacement for sync_packages().
                if lf['end'] <= prev_cam or lf['begin'] > cur_cam:
                    continue
                pts += lf['point_count']
            total_points += pts
            cut_segments.append({'previous_camera_time': prev_cam, 'current_camera_time': cur_cam, 'candidate_lidar_points': pts})
        summary = {
            'lidar': {
                'frame_count': len(self.lidar_frames),
                'header_time_first': lidar_headers[0] if lidar_headers else None,
                'header_time_last': lidar_headers[-1] if lidar_headers else None,
                'period_ms': stats(lidar_periods_ms),
                'scan_frequency_hz': ((len(lidar_headers)-1)/lidar_duration) if lidar_duration > 0 else None,
                'backwards': self.lidar_backwards,
                'duplicates': self.lidar_duplicates,
                'period_jump_gt20ms_from_100ms': self.lidar_jumps,
                'frame_duration_ms': stats([x['duration_ms'] for x in self.lidar_frames]),
            },
            'camera': {
                'frame_count': len(self.camera_frames),
                'period_ms': stats(cam_periods_ms),
                'frequency_hz': ((len(cam_times)-1)/cam_duration) if cam_duration > 0 else None,
            },
            'imu': {
                'sample_count': len(self.imu_times),
                'period_ms': stats(imu_periods_ms),
                'frequency_hz': ((len(self.imu_times)-1)/imu_duration) if imu_duration > 0 else None,
                'backwards': self.imu_backwards,
                'duplicates': self.imu_duplicates,
            },
            'camera_lidar_relation': {
                'rows': len(self.camera_lidar_rows),
                'camera_minus_lidar_begin_ms': stats([x['camera_minus_lidar_begin_ms'] for x in self.camera_lidar_rows]),
                'camera_minus_lidar_end_ms': stats([x['camera_minus_lidar_end_ms'] for x in self.camera_lidar_rows]),
                'jump_gt50ms_between_rows': sum(1 for a,b in zip(self.camera_lidar_rows,self.camera_lidar_rows[1:]) if abs(b['camera_minus_lidar_begin_ms']-a['camera_minus_lidar_begin_ms'])>50),
                'match_type_counts': dict((k, sum(1 for x in self.camera_lidar_rows if x['match_type'] == k)) for k in sorted(set(x['match_type'] for x in self.camera_lidar_rows))),
                'contains_camera_minus_lidar_begin_ms': stats([x['camera_minus_lidar_begin_ms'] for x in self.camera_lidar_rows if x['match_type'] == 'contains']),
                'contains_camera_minus_lidar_end_ms': stats([x['camera_minus_lidar_end_ms'] for x in self.camera_lidar_rows if x['match_type'] == 'contains']),
                'contains_begin_abs_p95_ms': percentile([abs(x['camera_minus_lidar_begin_ms']) for x in self.camera_lidar_rows if x['match_type'] == 'contains'], 95),
                'nearest_rows': [x for x in self.camera_lidar_rows if x['match_type'] != 'contains'][:20],
            },
            'lidar_per_point_offset': {
                'frames_with_nonzero_offsets': sum(1 for x in point_diag if x['offset_nonzero'] > 0),
                'frames_with_offset_backwards': sum(1 for x in point_diag if x['offset_backwards'] > 0),
                'frames_with_duplicate_offsets': sum(1 for x in point_diag if x['offset_duplicates'] > 0),
                'offset_max_ns': stats([x['offset_max_ns'] for x in point_diag]),
            },
            'imu_lidar_coverage': {
                'checked_lidar_frames': len(coverage),
                'frames_without_imu': sum(1 for x in coverage if x['imu_sample_count'] == 0),
                'frames_not_covering_begin': sum(1 for x in coverage if not x['covers_begin']),
                'frames_not_covering_end': sum(1 for x in coverage if not x['covers_end']),
                'imu_samples_per_lidar_frame': stats([x['imu_sample_count'] for x in coverage]),
            },
            'fast_livo_cut_interval_check': {
                'segments': len(cut_segments),
                'candidate_points_per_camera_segment': stats([x['candidate_lidar_points'] for x in cut_segments]),
                'note': 'External interval check for previous_camera_time < lidar_point_time <= current_camera_time. It does not modify or replace sync_packages().',
            },
            'samples': {
                'lidar_first_10': self.lidar_frames[:10],
                'camera_lidar_first_10': self.camera_lidar_rows[:10],
                'camera_lidar_last_10': self.camera_lidar_rows[-10:],
                'imu_coverage_first_10': coverage[:10],
            }
        }
        return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=35.0)
    ap.add_argument('--out', default='/home/nuc/versions/version5/logs/time_chain_validation.json')
    args = ap.parse_args()
    rospy.init_node('version5_time_chain_validator', anonymous=True)
    v = Validator(args.duration)
    rospy.Subscriber('/livox/lidar', CustomMsg, v.lidar_cb, queue_size=200)
    rospy.Subscriber('/livox/imu', Imu, v.imu_cb, queue_size=20000)
    rospy.Subscriber('/uvc_camera/image_sync', Image, v.image_cb, queue_size=200)
    start = time.time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.time() - start < args.duration:
        rate.sleep()
    result = v.finish()
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
