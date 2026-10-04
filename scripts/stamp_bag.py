#!/usr/bin/env python3
"""Rewrite a bag's zero-stamped scans with their recording timestamps.

The 2026-10-04 mapping bag was recorded by a bridge build that stamped
/odom and /tf but left /scan unstamped (header 0.0). SLAM needs scans on
the same timeline as the recorded TF, and the per-message recording
timestamp (bag_ts) is exactly that timeline.

Reads the source mcap, writes a sibling directory suffixed `-stamped`
with every /scan header replaced by the message's bag timestamp; all
other topics pass through unchanged.

Usage: stamp_bag.py SRC_BAG_DIR
"""
import sys
import shutil
from pathlib import Path

import rclpy.serialization as ser
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main(src_dir: str) -> None:
    src = Path(src_dir).resolve()
    dst = src.parent / (src.name + '-stamped')
    if dst.exists():
        shutil.rmtree(dst)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(src), storage_id='mcap'),
        ConverterOptions('cdr', 'cdr'),
    )

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(dst), storage_id='mcap'),
        ConverterOptions('cdr', 'cdr'),
    )
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)

    total = rescanned = 0
    while reader.has_next():
        topic, data, bag_ts = reader.read_next()
        changed = False
        if topic == '/scan':
            msg = ser.deserialize_message(data, LaserScan)
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                msg.header.stamp.sec = bag_ts // 1_000_000_000
                msg.header.stamp.nanosec = bag_ts % 1_000_000_000
                data = ser.serialize_message(msg)
                changed = True
            total += 1
        elif topic == '/tf':
            msg = ser.deserialize_message(data, TFMessage)
            restamped = False
            for tr in msg.transforms:
                if tr.header.stamp.sec == 0 and tr.header.stamp.nanosec == 0:
                    tr.header.stamp.sec = bag_ts // 1_000_000_000
                    tr.header.stamp.nanosec = bag_ts % 1_000_000_000
                    restamped = True
            if restamped:
                data = ser.serialize_message(msg)
                changed = True
        if changed:
            rescanned += 1
        writer.write(topic, data, bag_ts)

    print(f"scans: {total}, messages re-stamped: {rescanned} -> {dst}")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
