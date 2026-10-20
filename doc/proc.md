# Proc: Bag Overview

1. Add `BAG_PATH` constant (default `data/ibex_bags/highbay_state_est-9_24-1225`).
2. Open the bag with `rosbags.rosbag2.Reader`; read `duration`, `start_time`, `end_time`, `message_count`, `topics` (type, count). Storage format from `metadata.yaml`; size from summing the bag's data files.
3. Print bag basics (times in UTC).
4. Print topic table sorted by message count descending; Avg Hz = count / duration; empty topics marked `(empty)`.
5. Missing path or metadata: print a clear error and exit.
6. Run on one mcap bag and one db3 bag to confirm both load.

# Proc: Path Visualization

1. Dependencies: `pyproj` and `matplotlib` (already installed). You install `pyproj`; I'll note it for `requirements.txt` in `notes.md`.
2. Add `BAG_PATH` constant. Same section markers, docstrings, and full-word naming as `overview.py`.
3. Load: with `rosbags.highlevel.AnyReader`, read `/gps_values`, `/graph_pose`, `/odom_offset`, `/kiss/odometry` into numpy arrays using header stamps (seconds, relative to the first valid GPS stamp). Exit with a clear error if a topic has no messages.
4. GPS to UTM: drop fixes with quality 0 or lat/lon of 0. Pick the UTM zone from the first valid fix, then convert with `pyproj.Transformer`.
5. Graph to UTM: interpolate `/odom_offset` (easting, northing, unwrapped yaw) at each `graph_pose` stamp, then rotate and translate the pose XY. Z is unchanged. Rotate the XY covariance block by the same yaw. Drop poses before the first offset message.
6. KISS-ICP placement: apply one fixed transform to the raw, zero-initialized KISS path: `position = rotation(yaw) @ kiss_xy + (easting, northing)`.
   - Default values (no saved file): `fit_kiss_offset` returns the GPS heading fit (first GPS fix position, heading from the first 40 m of GPS travel compared with KISS over the same span).
   - Add `OFFSETS_DIR = Path("offsets")`; `load_kiss_offset` / `save_kiss_offset` read and write `offsets/<bag name>.json` (`easting`, `northing`, `yaw_degrees`).
   - Map window: three `TextBox` widgets (easting, northing, yaw) under the axes. On submit, parse the floats (invalid input is ignored and the old text restored), save the JSON, recompute KISS-ICP, and update its line and markers on the map and its error line in the comparison window. Keep widget references alive.
   - `plot_map` and `plot_comparison` return the artists the callback updates.
7. Error: interpolate graph and KISS onto the GPS timestamps over their common time range, then compute horizontal distance to GPS.
8. Plot: one figure, 2x2 panels, `plt.show()`.
   1. XY overlay, equal aspect, with start and end markers.
   2. Horizontal error vs time for graph and KISS.
   3. Z vs time for graph and KISS (GPS has no altitude).
   4. `graph_pose` path with 2-sigma covariance ellipses at evenly spaced poses.
9. Map window: split the XY overlay out of the comparison figure into its own large figure (about 12x10 in). Plot absolute UTM, then call `contextily.add_basemap` with the OpenStreetMap Mapnik provider and `crs=f"EPSG:{32600 + zone}"`. Return the zone from `build_gps_utm`. If tile download fails, warn and show the paths without a basemap.
10. Analysis window: one 3x3-grid figure with every non-map panel. Left column: error, Z, GPS quality. Middle: offset yaw, offset position, raw paths. Right: covariance ellipses spanning all rows. One shared legend.
11. Figure saving: add `FIGURES_DIR = Path("figures")` and `FIGURE_DPI = 150`, plus `save_figure(figure, name)` writing `figures/<bag name>_<name>.png` (warns on failure). Save both figures on launch, after each offset change, and from each figure's `close_event`.
12. Hand off for your testing. Test on one mcap bag with state estimation (for example `highbay_state_est-9_24-1225`).
