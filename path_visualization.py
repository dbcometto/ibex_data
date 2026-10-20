"""Plot and compare the GPS, factor-graph, and KISS-ICP paths of one ROS 2 bag in UTM."""

import json
import sys
from dataclasses import dataclass, replace
from enum import IntEnum
from pathlib import Path

import contextily
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from matplotlib.widgets import TextBox
from pyproj import Transformer
from rosbags.highlevel import AnyReader

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

# Bag to plot: change this one line
SELECTED_BAG = Bag.STATE_EST_1329
BAG_PATH = BAGS_DIRECTORY / BAG_DIRECTORIES[SELECTED_BAG]

TOPIC_GPS = "/gps_values"
TOPIC_GRAPH = "/graph_pose"
TOPIC_OFFSET = "/odom_offset"
TOPIC_KISS = "/kiss/odometry"

# Per-bag KISS-ICP placement is saved to OFFSETS_DIR/<bag name>.json
OFFSETS_DIR = Path("offsets")

# Figures are saved to FIGURES_DIR/<bag name>_<map|analysis>.png
FIGURES_DIR = Path("figures")
FIGURE_DPI = 150

# GPS travel from the first fix used for the default KISS-ICP heading fit (the vehicle may start stationary)
ALIGN_DISTANCE_METERS = 40.0

# Basemap tiles: OpenStreetMap's policy requires a user-agent that identifies the app
# (kept anonymous; if their servers still block it, switch to contextily.providers.CartoDB.Positron)
BASEMAP_SOURCE = contextily.providers.OpenStreetMap.Mapnik
TILE_HEADERS = {"user-agent": "ibex-data-path-visualization/1.0 (personal research script)"}

# Covariance ellipse display: count along the path, sigma level, and visual exaggeration
ELLIPSE_COUNT = 30
ELLIPSE_SIGMA = 2.0
ELLIPSE_SCALE = 10.0

# One fixed color per source so every panel agrees
SOURCE_COLORS = {
    "GPS": "tab:blue",
    "Graph": "tab:orange",
    "Graph (fixed offset)": "tab:cyan",
    "KISS-ICP": "tab:green",
}

# Map traces placed by the user-set fixed offset (they move when the offset boxes change)
FIXED_OFFSET_LABELS = ("Graph (fixed offset)", "KISS-ICP")


#==========# DATA TYPES #==========#

@dataclass
class Trajectory:
    """Timestamped poses: stamps (N,), positions (N, 3), yaws (N,), xy covariances (N, 2, 2)."""

    stamps: np.ndarray
    positions: np.ndarray
    yaws: np.ndarray
    covariances: np.ndarray


#==========# LOADING #==========#

def load_messages(bag_path, topics):
    """Read all messages on the given topics and return them as {topic: [message, ...]}."""
    messages = {topic: [] for topic in topics}
    with AnyReader([bag_path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic in topics]
        for connection, _, raw in reader.messages(connections=connections):
            messages[connection.topic].append(reader.deserialize(raw, connection.msgtype))

    # Every topic is required
    for topic, topic_messages in messages.items():
        if not topic_messages:
            sys.exit(f"Error: no messages on {topic} in {bag_path}")
    return messages


def stamp_seconds(header):
    """Return a message header stamp as float seconds."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def quaternion_to_yaw(orientation):
    """Return the yaw angle (radians) of a quaternion message."""
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
    return np.arctan2(siny_cosp, cosy_cosp)


def build_trajectory(messages):
    """Build a Trajectory from PoseWithCovarianceStamped or Odometry messages."""
    stamps = np.array([stamp_seconds(message.header) for message in messages])
    poses = [message.pose.pose for message in messages]
    positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in poses])
    yaws = np.array([quaternion_to_yaw(pose.orientation) for pose in poses])

    # Keep only the xy block of the 6x6 row-major covariance
    covariances = np.array([np.reshape(message.pose.covariance, (6, 6))[:2, :2] for message in messages])
    return Trajectory(stamps, positions, yaws, covariances)


def build_gps_utm(messages):
    """Return (stamps, utm_xy, utm_crs) of valid GPS fixes converted to UTM, with the zone from the first fix."""
    valid = [
        message for message in messages
        if message.auto_gps_quality > 0 and message.auto_gps_lat != 0.0 and message.auto_gps_lon != 0.0
    ]
    if not valid:
        sys.exit("Error: no valid GPS fixes (quality > 0 and non-zero lat/lon)")

    stamps = np.array([stamp_seconds(message.header) for message in valid])
    latitudes = np.array([message.auto_gps_lat for message in valid])
    longitudes = np.array([message.auto_gps_lon for message in valid])

    # UTM zone (northern hemisphere) from the first fix
    zone = int((longitudes[0] + 180) // 6) + 1
    utm_crs = f"EPSG:{32600 + zone}"
    transformer = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    east, north = transformer.transform(longitudes, latitudes)
    return stamps, np.column_stack([east, north]), utm_crs


#==========# ALIGNMENT #==========#

def interpolate_xy(stamps, trajectory):
    """Return the trajectory xy interpolated at the given stamps, shape (len(stamps), 2)."""
    return np.column_stack([np.interp(stamps, trajectory.stamps, trajectory.positions[:, axis]) for axis in (0, 1)])


def graph_to_utm(graph, offset):
    """Transform graph poses into UTM using the interpolated odom-to-UTM SE(2) offset."""
    # Offset is only known from its first message onward
    keep = graph.stamps >= offset.stamps[0]
    stamps = graph.stamps[keep]
    offset_xy = interpolate_xy(stamps, offset)
    offset_yaw = np.interp(stamps, offset.stamps, np.unwrap(offset.yaws))

    # Rotate by the offset yaw, then translate by the offset position
    cos_yaw, sin_yaw = np.cos(offset_yaw), np.sin(offset_yaw)
    rotations = np.stack([np.stack([cos_yaw, -sin_yaw], axis=-1), np.stack([sin_yaw, cos_yaw], axis=-1)], axis=-2)
    graph_xy = graph.positions[keep, :2]
    utm_xy = offset_xy + np.einsum("nij,nj->ni", rotations, graph_xy)

    positions = np.column_stack([utm_xy, graph.positions[keep, 2]])
    covariances = rotations @ graph.covariances[keep] @ rotations.transpose(0, 2, 1)
    return Trajectory(stamps, positions, graph.yaws[keep] + offset_yaw, covariances)


def place_with_offset(trajectory, easting, northing, yaw_degrees):
    """Place a zero-initialized local trajectory in UTM with one fixed transform (yaw 0 = east, counter-clockwise positive)."""
    angle = np.radians(yaw_degrees)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

    positions = np.column_stack([trajectory.positions[:, :2] @ rotation.T + [easting, northing], trajectory.positions[:, 2]])
    covariances = rotation @ trajectory.covariances @ rotation.T
    return replace(trajectory, positions=positions, yaws=trajectory.yaws + angle, covariances=covariances)


def fit_kiss_offset(kiss, gps_stamps, gps_xy):
    """Return the default (easting, northing, yaw_degrees) from a GPS heading fit at motion onset."""
    # First GPS fix far enough from the start to give a stable heading (the vehicle may sit still first)
    travel = np.hypot(*(gps_xy - gps_xy[0]).T)
    reached = np.flatnonzero(travel >= ALIGN_DISTANCE_METERS)
    if reached.size == 0:
        sys.exit(f"Error: GPS never traveled {ALIGN_DISTANCE_METERS} m, cannot place KISS-ICP")
    onset_index = reached[0]

    # Heading of the same time span in GPS and in KISS-ICP
    kiss_start, kiss_onset = interpolate_xy([gps_stamps[0], gps_stamps[onset_index]], kiss)
    gps_delta = gps_xy[onset_index] - gps_xy[0]
    kiss_delta = kiss_onset - kiss_start
    if np.hypot(*kiss_delta) == 0.0:
        sys.exit("Error: KISS-ICP did not move between the first GPS fix and motion onset")
    angle = np.arctan2(gps_delta[1], gps_delta[0]) - np.arctan2(kiss_delta[1], kiss_delta[0])

    # Rotate about the KISS position at the first fix, then place that point on the first GPS fix
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    easting, northing = gps_xy[0] - rotation @ kiss_start
    return float(easting), float(northing), float(np.degrees(angle))


def offset_file_path(bag_name):
    """Return the JSON path holding the saved KISS-ICP offset for a bag."""
    return OFFSETS_DIR / f"{bag_name}.json"


def load_kiss_offset(bag_name):
    """Return the saved (easting, northing, yaw_degrees) for a bag, or None if missing or unreadable."""
    path = offset_file_path(bag_name)
    if not path.is_file():
        return None
    try:
        saved = json.loads(path.read_text())
        return float(saved["easting"]), float(saved["northing"]), float(saved["yaw_degrees"])
    except (ValueError, KeyError, TypeError) as error:
        print(f"Warning: ignoring unreadable {path} ({error})")
        return None


def save_kiss_offset(bag_name, values):
    """Save (easting, northing, yaw_degrees) for a bag to its JSON file."""
    path = offset_file_path(bag_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    easting, northing, yaw_degrees = values
    path.write_text(json.dumps({"easting": easting, "northing": northing, "yaw_degrees": yaw_degrees}, indent=2))


def horizontal_error(trajectory, gps_stamps, gps_xy):
    """Return (stamps, error) of horizontal distance to GPS over the trajectory's time range."""
    inside = (gps_stamps >= trajectory.stamps[0]) & (gps_stamps <= trajectory.stamps[-1])
    estimate_xy = interpolate_xy(gps_stamps[inside], trajectory)
    return gps_stamps[inside], np.hypot(*(estimate_xy - gps_xy[inside]).T)


#==========# PLOTTING #==========#

def add_covariance_ellipses(axis, trajectory):
    """Draw sigma-level covariance ellipses at evenly spaced poses, exaggerated by ELLIPSE_SCALE."""
    indices = np.unique(np.linspace(0, len(trajectory.stamps) - 1, ELLIPSE_COUNT).astype(int))
    for index in indices:
        # Principal axes from the eigen decomposition (eigh sorts ascending)
        eigenvalues, eigenvectors = np.linalg.eigh(trajectory.covariances[index])
        radii = ELLIPSE_SIGMA * ELLIPSE_SCALE * np.sqrt(np.clip(eigenvalues, 0.0, None))
        angle = np.degrees(np.arctan2(eigenvectors[1, 1], eigenvectors[0, 1]))
        axis.add_patch(Ellipse(
            trajectory.positions[index, :2], 2 * radii[1], 2 * radii[0], angle=angle,
            fill=False, edgecolor="tab:red",
        ))


class OffsetControls:
    """Easting, northing, and yaw text boxes; submitting any of them calls on_change with the new values."""

    # (label, [left, bottom, width, height]) in figure coordinates
    LAYOUT = (
        ("Easting (m)", [0.09, 0.015, 0.22, 0.04]),
        ("Northing (m)", [0.46, 0.015, 0.22, 0.04]),
        ("Yaw (deg)", [0.83, 0.015, 0.10, 0.04]),
    )

    def __init__(self, figure, values, on_change):
        """Add the three boxes to the figure, starting at the given (easting, northing, yaw_degrees)."""
        self.values = tuple(values)
        self.on_change = on_change
        self.boxes = []
        for (label, rectangle), value in zip(self.LAYOUT, self.values):
            box = TextBox(figure.add_axes(rectangle), label, initial=f"{value:.3f}")
            box.on_submit(self.submit)
            self.boxes.append(box)

    def submit(self, _text):
        """Parse all boxes; on success store and report the values, otherwise restore the last good ones."""
        try:
            values = tuple(float(box.text) for box in self.boxes)
        except ValueError:
            print("Warning: offset boxes must hold numbers; restoring previous values")
            self.show(self.values)
            return
        self.values = values
        self.on_change(values)

    def show(self, values):
        """Write values into the boxes without triggering submit."""
        for box, value in zip(self.boxes, values):
            box.eventson = False
            box.set_val(f"{value:.3f}")
            box.eventson = True


def plot_map(gps_xy, graph, graph_fixed, kiss, utm_crs):
    """Show a large top-down overlay of the paths in absolute UTM over a street basemap.

    Returns the figure and {label: (line, start marker, end marker)} for the traces placed by the
    fixed offset, for later updates.
    """
    figure, axis = plt.subplots(figsize=(10.5, 8.5))

    # Paths with start (circle) and end (square) markers
    paths = (
        ("GPS", gps_xy),
        ("Graph", graph.positions[:, :2]),
        ("Graph (fixed offset)", graph_fixed.positions[:, :2]),
        ("KISS-ICP", kiss.positions[:, :2]),
    )
    fixed_offset_artists = {}
    for label, path in paths:
        line, = axis.plot(path[:, 0], path[:, 1], label=label, color=SOURCE_COLORS[label])
        start, = axis.plot(*path[0], "o", color=SOURCE_COLORS[label], markeredgecolor="black")
        end, = axis.plot(*path[-1], "s", color=SOURCE_COLORS[label], markeredgecolor="black")
        if label in FIXED_OFFSET_LABELS:
            fixed_offset_artists[label] = (line, start, end)
    axis.set_title(f"{BAG_PATH.name}: path in UTM (circle = start, square = end; KISS-ICP and Graph (fixed offset) use the boxes below)")
    axis.set_xlabel("Easting (m)")
    axis.set_ylabel("Northing (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.margins(0.1)
    axis.legend()

    # Street basemap tiles warped into the UTM frame; the paths still plot without it
    try:
        contextily.add_basemap(axis, crs=utm_crs, source=BASEMAP_SOURCE, headers=TILE_HEADERS)
    except Exception as error:
        print(f"Warning: basemap unavailable ({error}); showing paths only")

    # Leave room at the bottom for the offset text boxes
    figure.tight_layout(rect=(0, 0.07, 1, 1))
    return figure, fixed_offset_artists


def plot_analysis(gps_stamps, gps_xy, gps_quality_stamps, gps_quality, graph, graph_raw, kiss, kiss_raw, offset):
    """Show one analysis window: error, Z, GPS quality, offset yaw/position, raw paths, graph covariance.

    Returns the figure and the KISS-ICP error line for later updates.
    """
    origin = gps_xy[0]
    start_time = gps_stamps[0]
    graph_local = graph.positions[:, :2] - origin

    # Three columns: time series on the left, offset diagnostics and raw paths in the middle,
    # covariance ellipses spanning the right column
    figure = plt.figure(figsize=(15, 8.5))
    grid = figure.add_gridspec(3, 3)
    axis_error = figure.add_subplot(grid[0, 0])
    axis_height = figure.add_subplot(grid[1, 0])
    axis_quality = figure.add_subplot(grid[2, 0])
    axis_yaw = figure.add_subplot(grid[0, 1])
    axis_offset = figure.add_subplot(grid[1, 1])
    axis_raw = figure.add_subplot(grid[2, 1])
    axis_ellipse = figure.add_subplot(grid[:, 2])

    # Panel 1: horizontal error against GPS
    for label, trajectory in (("Graph", graph), ("KISS-ICP", kiss)):
        stamps, error = horizontal_error(trajectory, gps_stamps, gps_xy)
        line, = axis_error.plot(stamps - start_time, error, label=label, color=SOURCE_COLORS[label])
        if label == "KISS-ICP":
            kiss_error_line = line
    axis_error.set_title("Horizontal error vs GPS")
    axis_error.set_xlabel("Time from first fix (s)")
    axis_error.set_ylabel("Error (m)")
    axis_error.grid(True)

    # Panel 2: height (GPS has no altitude)
    for label, trajectory in (("Graph", graph), ("KISS-ICP", kiss)):
        axis_height.plot(
            trajectory.stamps - start_time, trajectory.positions[:, 2] - trajectory.positions[0, 2],
            label=label, color=SOURCE_COLORS[label],
        )
    axis_height.set_title("Z from first pose")
    axis_height.set_xlabel("Time from first fix (s)")
    axis_height.set_ylabel("Z (m)")
    axis_height.grid(True)

    # Panel 3: graph covariance ellipses
    axis_ellipse.plot(graph_local[:, 0], graph_local[:, 1], color=SOURCE_COLORS["Graph"])
    add_covariance_ellipses(axis_ellipse, replace(graph, positions=np.column_stack([graph_local, graph.positions[:, 2]])))
    axis_ellipse.autoscale_view()
    axis_ellipse.set_title(f"Graph {ELLIPSE_SIGMA:g}-sigma covariance (ellipses scaled x{ELLIPSE_SCALE:g})")
    axis_ellipse.set_xlabel("East from first fix (m)")
    axis_ellipse.set_ylabel("North from first fix (m)")
    axis_ellipse.set_aspect("equal", adjustable="datalim")
    axis_ellipse.grid(True)

    # Diagnostic panels
    draw_diagnostics(
        axis_raw, axis_yaw, axis_offset, axis_quality,
        start_time, gps_quality_stamps, gps_quality, graph_raw, kiss_raw, offset,
    )

    # Single shared legend for the Graph / KISS-ICP colors, then pack the panels to fill the figure
    handles, labels = axis_error.get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=len(labels))
    figure.suptitle(f"{BAG_PATH.name} analysis")
    figure.tight_layout(rect=(0, 0.04, 1, 1))
    return figure, kiss_error_line


def draw_diagnostics(axis_raw, axis_yaw, axis_offset, axis_quality, start_time, gps_quality_stamps, gps_quality, graph_raw, kiss_raw, offset):
    """Draw raw (unaligned) paths, odom offset yaw and position, and GPS quality onto the given axes."""
    # Raw paths in their own frames: graph in aligned_odom, KISS-ICP in odom
    for label, trajectory in (("Graph", graph_raw), ("KISS-ICP", kiss_raw)):
        axis_raw.plot(
            trajectory.positions[:, 0] - trajectory.positions[0, 0],
            trajectory.positions[:, 1] - trajectory.positions[0, 1],
            label=label, color=SOURCE_COLORS[label],
        )
    axis_raw.set_title("Raw paths, no alignment (own frames)")
    axis_raw.set_xlabel("x from first pose (m)")
    axis_raw.set_ylabel("y from first pose (m)")
    axis_raw.set_aspect("equal", adjustable="datalim")
    axis_raw.grid(True)

    # Odom-to-UTM offset yaw
    offset_time = offset.stamps - start_time
    axis_yaw.plot(offset_time, np.degrees(np.unwrap(offset.yaws)), color="tab:purple")
    axis_yaw.set_title("/odom_offset yaw")
    axis_yaw.set_xlabel("Time from first fix (s)")
    axis_yaw.set_ylabel("Yaw (deg, unwrapped)")
    axis_yaw.grid(True)

    # Odom-to-UTM offset position
    axis_offset.plot(offset_time, offset.positions[:, 0] - offset.positions[0, 0], label="Easting", color="tab:purple")
    axis_offset.plot(offset_time, offset.positions[:, 1] - offset.positions[0, 1], label="Northing", color="tab:brown", linestyle="--")
    axis_offset.set_title("/odom_offset position")
    axis_offset.set_xlabel("Time from first fix (s)")
    axis_offset.set_ylabel("Change from first message (m)")
    axis_offset.legend()
    axis_offset.grid(True)

    # GPS fix quality, including invalid fixes
    axis_quality.plot(gps_quality_stamps - start_time, gps_quality, ".", markersize=3, color=SOURCE_COLORS["GPS"])
    axis_quality.set_title("GPS quality (0 = invalid)")
    axis_quality.set_xlabel("Time from first fix (s)")
    axis_quality.set_ylabel("auto_gps_quality")
    axis_quality.grid(True)


def save_figure(figure, name):
    """Save a figure as FIGURES_DIR/<bag name>_<name>.png, warning instead of failing on error."""
    path = FIGURES_DIR / f"{BAG_PATH.name}_{name}.png"
    try:
        FIGURES_DIR.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=FIGURE_DPI)
    except Exception as error:
        print(f"Warning: could not save {path} ({error})")


#==========# MAIN #==========#

def main():
    """Load the selected bag, align the three paths in UTM, and plot the comparison."""
    messages = load_messages(BAG_PATH, [TOPIC_GPS, TOPIC_GRAPH, TOPIC_OFFSET, TOPIC_KISS])

    gps_stamps, gps_xy, utm_crs = build_gps_utm(messages[TOPIC_GPS])
    graph_raw = build_trajectory(messages[TOPIC_GRAPH])
    kiss_raw = build_trajectory(messages[TOPIC_KISS])
    offset = build_trajectory(messages[TOPIC_OFFSET])
    graph = graph_to_utm(graph_raw, offset)

    # KISS-ICP placement: the saved per-bag offset, else the GPS heading fit
    kiss_offset = load_kiss_offset(BAG_PATH.name) or fit_kiss_offset(kiss_raw, gps_stamps, gps_xy)
    kiss = place_with_offset(kiss_raw, *kiss_offset)
    graph_fixed = place_with_offset(graph_raw, *kiss_offset)

    # All GPS fixes (valid or not) for the quality diagnostic
    gps_quality_stamps = np.array([stamp_seconds(message.header) for message in messages[TOPIC_GPS]])
    gps_quality = np.array([message.auto_gps_quality for message in messages[TOPIC_GPS]])

    map_figure, fixed_offset_artists = plot_map(gps_xy, graph, graph_fixed, kiss, utm_crs)
    analysis_figure, kiss_error_line = plot_analysis(
        gps_stamps, gps_xy, gps_quality_stamps, gps_quality, graph, graph_raw, kiss, kiss_raw, offset,
    )

    figures = {"map": map_figure, "analysis": analysis_figure}

    def save_all_figures():
        """Save every figure to its file."""
        for name, figure in figures.items():
            save_figure(figure, name)

    def on_offset_change(values):
        """Save the new offset, move the fixed-offset traces and the KISS-ICP error line, and re-save the figures."""
        save_kiss_offset(BAG_PATH.name, values)
        for label, raw_trajectory in (("KISS-ICP", kiss_raw), ("Graph (fixed offset)", graph_raw)):
            moved = place_with_offset(raw_trajectory, *values)
            path = moved.positions[:, :2]
            line, start, end = fixed_offset_artists[label]
            line.set_data(path[:, 0], path[:, 1])
            start.set_data([path[0, 0]], [path[0, 1]])
            end.set_data([path[-1, 0]], [path[-1, 1]])
            if label == "KISS-ICP":
                kiss_moved = moved

        stamps, error = horizontal_error(kiss_moved, gps_stamps, gps_xy)
        kiss_error_line.set_data(stamps - gps_stamps[0], error)
        kiss_error_line.axes.relim()
        kiss_error_line.axes.autoscale_view()
        map_figure.canvas.draw_idle()
        analysis_figure.canvas.draw_idle()
        save_all_figures()
        print(f"Saved {offset_file_path(BAG_PATH.name)}: easting {values[0]:.3f}, northing {values[1]:.3f}, yaw {values[2]:.3f} deg")

    # Keep a reference so the text boxes stay responsive
    controls = OffsetControls(map_figure, kiss_offset, on_offset_change)

    # Save on launch (after the controls exist, so the map image shows the offset boxes) and again on close
    save_all_figures()
    for name, figure in figures.items():
        figure.canvas.mpl_connect("close_event", lambda event, figure=figure, name=name: save_figure(figure, name))
    plt.show()


if __name__ == "__main__":
    main()
