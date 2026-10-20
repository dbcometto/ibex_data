# Spec: Bag Overview

## Mission
`overview.py` prints a text summary of one selected ROS 2 bag from the ATV recordings in `data/ibex_bags/`.

## Architecture
- Single script, `overview.py`.
- Bag is selected by a `BAG_PATH` constant at the top of the file.
- Reads metadata only via `rosbags` (mcap and db3). No message payloads are deserialized.
- Output is plain terminal text with two sections:
  1. Bag basics: storage format, size, start/end time, duration, total messages.
  2. Topic table: name, type, message count, average Hz. Empty topics are kept and flagged.
- Out of scope: message-derived stats, health checks, plots, CLI args.

# Spec: Path Visualization

## Mission
`path_visualization.py` plots and compares the path from three sources for one selected bag: the factor-graph fused estimate, KISS-ICP odometry, and GPS.

## Architecture
- Single script, `path_visualization.py`. Bag selected by the `SELECTED_BAG` constant, a `Bag` IntEnum of the recorded bags (mapped to their directories).
- Reads messages via `rosbags`. Time comes from message header stamps.
- Common frame is UTM. GPS (`/gps_values`) is the reference: invalid fixes are dropped and lat/lon is converted to UTM with `pyproj`.
- `/graph_pose` (`aligned_odom`) is transformed to UTM using the SE(2) `/odom_offset` (odom origin in UTM), interpolated at each timestamp.
- `/kiss/odometry` is zero-initialized and stays raw. It is placed in UTM with one fixed, user-set transform (absolute easting, northing, yaw in degrees, ROS convention). `/odom_offset` is not used for KISS-ICP because its yaw did not converge in the test bag. Drift is left visible.
- The raw `/graph_pose` (`aligned_odom`) is also drawn on the map, placed with the same fixed offset, as "Graph (fixed offset)", next to the time-varying-offset Graph trace.
- The transform is edited in text boxes on the map window and saved per bag to `offsets/<bag name>.json` in the repo. With no saved file, the boxes start from a heading fit to GPS (first fix position, heading from the first 40 m of GPS travel). Submitting a value saves it and redraws KISS-ICP on the map and in the error panel.
- Output is two interactive matplotlib windows (the map, and one analysis window holding every other panel):
  1. Map: large top-down XY overlay of the three paths in absolute UTM, with start and end markers, over an OpenStreetMap street basemap (`contextily`, tiles downloaded and cached).
  2. Analysis (seven panels): position error vs time against GPS, Z vs time, GPS quality, `/odom_offset` yaw and position, raw unaligned paths, and `graph_pose` covariance ellipses along the path (relative metres, no basemap).
- Both figures are saved as PNG (150 dpi) to `figures/<bag name>_map.png` and `figures/<bag name>_analysis.png`: on launch, after each offset change, and on window close. Files are overwritten.
- Out of scope: multi-bag comparison, satellite imagery.
