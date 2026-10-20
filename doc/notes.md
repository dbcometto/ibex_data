# Notes

## Setup
- Python via `.venv`. Dependencies: `rosbags`, `numpy`, `matplotlib`, `ruamel.yaml`, plus `pyproj` and `contextily` (installed by hand).
- `requirements.txt` is saved as UTF-16, which `pip install -r` may reject. It also lacks `pyproj` and `contextily`.
- Run scripts from the repo root; `data/`, `offsets/` and `figures/` paths are relative to the working directory.

## overview.py
- Prints bag basics and a topic table (count, average Hz, empty topics flagged) from metadata only; no messages are read.
- Bag set by the `BAG_PATH` constant. It has not been moved to the `Bag` enum yet.

## path_visualization.py
Plots GPS, the factor-graph estimate (`/graph_pose`), and KISS-ICP (`/kiss/odometry`) in UTM for one bag.

**Bag selection:** `SELECTED_BAG = Bag.<NAME>` (an `IntEnum` mapped to the bag directories). Only the state-estimation bags (`STATE_EST_1225`, `STATE_EST_1329`) are expected to have all four required topics; the script exits with a clear error on any missing topic.

**Data handling**
- Time comes from message header stamps.
- GPS fixes with quality 0 or lat/lon of 0 are dropped, then converted to UTM with `pyproj` (zone from the first fix).
- Graph (orange): `/graph_pose` transformed with `/odom_offset`, interpolated at each stamp, with the XY covariance rotated to match.
- KISS-ICP (green) and Graph (fixed offset, cyan): the raw, zero-initialized paths placed by one fixed easting, northing and yaw (degrees, 0 = east, counter-clockwise positive).

**Offset boxes and saved files**
- Three text boxes on the map window set the fixed offset. Enter saves `offsets/<bag name>.json`, moves the KISS-ICP and cyan traces, and updates the KISS-ICP line in the error panel.
- With no saved file, the boxes start from a fit to GPS: first fix position and heading from the first 40 m of GPS travel (`ALIGN_DISTANCE_METERS`).
- Both figures save as `figures/<bag name>_map.png` and `_analysis.png` on launch, after each offset change, and on close.

**Windows**
1. Map: street basemap (OpenStreetMap through `contextily`, anonymous user-agent) with the paths, start and end markers, and the offset boxes.
2. Analysis (7 panels): error vs GPS, Z, GPS quality, `/odom_offset` yaw and position, raw unaligned paths, graph covariance ellipses (exaggerated x10).

## Findings (bag `highbay_state_est-9_24-1225`)
- The vehicle was stationary for about the first 47 s, so the first few metres of GPS travel are noise. This is why the heading fit uses a 40 m baseline.
- The GPS bearing once moving is about 22-30 degrees (north-northeast); the run should head roughly northeast.
- `/odom_offset` is not a usable constant here: its yaw wanders from about -76 to +96 degrees and its position drifts about 275 m. Neither the first, median nor last offset rotates KISS-ICP onto the GPS heading (that needs about +120 degrees).
- The raw graph and raw KISS-ICP headings agree in their own frames, which points at the offset estimate, not the lidar odometry. The jagged graph path in UTM is probably the wandering offset yaw applied to a smooth path (not yet confirmed from the raw-paths panel).
- Graph Z swings by tens of metres while KISS-ICP Z stays near 0. This is in the recorded `/graph_pose`, not caused by the plotting.
- The GPS track itself is poor in places; the error panel treats it as truth.

## Status and open items
- Not run by the assistant: the code was written and read back, and testing was done by the user in their own runs.
- The close-time figure save may fail on some matplotlib backends; it prints a warning instead of crashing.
- The cyan trace has no line in the error panel yet.
- Possible next steps: filter or color GPS by quality, share the `Bag` enum with `overview.py`, fix `requirements.txt`, and decide whether `offsets/` and `figures/` belong in git.
