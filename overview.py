"""Print a summary overview (bag basics and topic table) of one ROS 2 bag."""

import sys
from datetime import datetime, timezone
from pathlib import Path
from enum import IntEnum

from rosbags.rosbag2 import Reader
from ruamel.yaml import YAML

#==========# CONFIG #==========#
BAGS_DIRECTORY = Path("data/ibex_bags")

class Bag(IntEnum):
    """Recorded bags, numbered in recording order (names end in the start time, HHMM)."""
    STATE_EST_1225 = 1
    INSTA360_TEST_1233 = 2
    INSTA_TEST_1239 = 3
    INSTA360_NO_MCAP_1247 = 4
    INSTA360_TEST_1317 = 5
    STATE_EST_1329 = 6

BAG_DIRECTORIES = {
    Bag.STATE_EST_1225: "highbay_state_est-9_24-1225",
    Bag.INSTA360_TEST_1233: "highbay_insta360test_9_24_1233",
    Bag.INSTA_TEST_1239: "highbay_instatest-9_24-1239",
    Bag.INSTA360_NO_MCAP_1247: "insta360testnomcap-9_24-1247",
    Bag.INSTA360_TEST_1317: "insta360test-9_24_117",
    Bag.STATE_EST_1329: "highbay_stateest-924-129",
}

# Bag directory to summarize (must contain metadata.yaml)
SELECTED_BAG = Bag.STATE_EST_1329
BAG_PATH = BAGS_DIRECTORY / BAG_DIRECTORIES[SELECTED_BAG]

NANOSECONDS_PER_SECOND = 1e9


#==========# FORMATTING #==========#

def format_time(nanoseconds):
    """Return a UTC timestamp string for nanoseconds since the epoch."""
    moment = datetime.fromtimestamp(nanoseconds / NANOSECONDS_PER_SECOND, tz=timezone.utc)
    return moment.strftime("%Y-%m-%d %H:%M:%S UTC")


def format_duration(nanoseconds):
    """Return a duration string as seconds and h:mm:ss."""
    seconds = nanoseconds / NANOSECONDS_PER_SECOND
    hours, remainder = divmod(int(seconds), 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{seconds:.1f} s ({hours}:{minutes:02d}:{secs:02d})"


def format_size(byte_count):
    """Return a human-readable size string for a byte count."""
    for unit in ("B", "KB", "MB"):
        if byte_count < 1024:
            return f"{byte_count:.1f} {unit}"
        byte_count /= 1024
    return f"{byte_count:.1f} GB"


#==========# LOADING #==========#

def load_bag_summary(bag_path):
    """Read bag metadata only (no message payloads) and return it as a dict."""
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        sys.exit(f"Error: no metadata.yaml found in {bag_path}")

    # Storage format and file list come straight from metadata.yaml
    info = YAML(typ="safe").load(metadata_path)["rosbag2_bagfile_information"]
    size_bytes = sum((bag_path / name).stat().st_size for name in info["relative_file_paths"])

    # Timing and topic counts come from the rosbags reader
    with Reader(bag_path) as reader:
        topics = sorted(reader.topics.items(), key=lambda item: item[1].msgcount, reverse=True)
        return {
            "storage": info["storage_identifier"],
            "size_bytes": size_bytes,
            "start_time": reader.start_time,
            "end_time": reader.end_time,
            "duration": reader.duration,
            "message_count": reader.message_count,
            "topics": topics,
        }


#==========# DISPLAY #==========#

def print_basics(bag_path, summary):
    """Print the bag basics section."""
    empty_count = sum(1 for _, topic in summary["topics"] if topic.msgcount == 0)
    print(f"Bag:       {bag_path.name}")
    print(f"Path:      {bag_path}")
    print(f"Storage:   {summary['storage']}")
    print(f"Size:      {format_size(summary['size_bytes'])}")
    print(f"Start:     {format_time(summary['start_time'])}")
    print(f"End:       {format_time(summary['end_time'])}")
    print(f"Duration:  {format_duration(summary['duration'])}")
    print(f"Messages:  {summary['message_count']}")
    print(f"Topics:    {len(summary['topics'])} ({empty_count} empty)")


def print_topic_table(summary):
    """Print the topic table with message counts and average rates."""
    topics = summary["topics"]
    duration_seconds = summary["duration"] / NANOSECONDS_PER_SECOND
    name_width = max(len("Topic"), *(len(name) for name, _ in topics))
    type_width = max(len("Type"), *(len(topic.msgtype) for _, topic in topics))

    # Header
    print(f"{'Topic':<{name_width}}  {'Type':<{type_width}}  {'Messages':>9}  {'Avg Hz':>8}")
    print(f"{'-' * name_width}  {'-' * type_width}  {'-' * 9}  {'-' * 8}")

    # One row per topic, empty topics flagged
    for name, topic in topics:
        rate = topic.msgcount / duration_seconds if duration_seconds else 0.0
        flag = "  (empty)" if topic.msgcount == 0 else ""
        print(f"{name:<{name_width}}  {topic.msgtype:<{type_width}}  {topic.msgcount:>9}  {rate:>8.2f}{flag}")


#==========# MAIN #==========#

def main():
    """Load the selected bag and print its overview."""
    summary = load_bag_summary(BAG_PATH)
    print_basics(BAG_PATH, summary)
    print()
    print_topic_table(summary)


if __name__ == "__main__":
    main()
