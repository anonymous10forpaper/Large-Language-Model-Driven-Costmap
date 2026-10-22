

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any, Dict, Iterable, List, Mapping, Tuple

import yaml


Waypoint = Tuple[float, float, float]
WAYPOINT_NAME_RE = re.compile(r"WP0*([1-9][0-9]*)", re.IGNORECASE)
ZONE_NAME_RE = re.compile(r"[A-Z]", re.IGNORECASE)
DEFAULT_FACTORY_WAYPOINTS: Dict[str, Waypoint] = {
    "WP1": (-0.2, 0.41, 0.0),
    "WP2": (32.2, 0.41, 0.0),
    "WP3": (-0.2, -4.56, 0.0),
    "WP4": (32.2, -4.56, 0.0),
    "WP5": (-0.2, -13.40, 0.0),
    "WP6": (32.2, -13.40, 0.0),
    "WP7": (-0.2, -18.10, 0.0),
    "WP8": (32.2, -18.10, 0.0),
}


@dataclass(frozen=True)
class ZoneBounds:


    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center(self) -> Tuple[float, float]:
        return (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
        )

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return self.x_min, self.y_min, self.x_max, self.y_max


DEFAULT_FACTORY_ZONES: Dict[str, ZoneBounds] = {
    "A": ZoneBounds(8.01, -1.55, 23.96, 1.90),
    "B": ZoneBounds(8.01, -6.05, 24.01, -2.60),
    "C": ZoneBounds(8.01, -10.70, 24.06, -7.20),
    "D": ZoneBounds(8.01, -15.20, 24.01, -11.70),
    "E": ZoneBounds(8.01, -19.70, 24.01, -16.20),
}


@dataclass(frozen=True)
class MapAnnotations:


    waypoints: Dict[str, Waypoint]
    zones: Dict[str, ZoneBounds]
    zones_defined: bool


def normalize_waypoint_name(value: Any) -> str:

    match = WAYPOINT_NAME_RE.fullmatch(str(value or "").strip())
    if match is None:
        raise ValueError("Waypoint names must use WP followed by a number")
    return f"WP{int(match.group(1))}"


def waypoint_sort_key(name: str) -> Tuple[int, str]:

    try:
        canonical = normalize_waypoint_name(name)
        return int(canonical[2:]), canonical
    except ValueError:
        return 10**9, str(name)


def normalize_waypoints(values: Mapping[str, Any]) -> Dict[str, Waypoint]:

    normalized: Dict[str, Waypoint] = {}
    for raw_name, raw_value in values.items():
        name = normalize_waypoint_name(raw_name)
        if isinstance(raw_value, Mapping):
            parts = (
                raw_value.get("x"),
                raw_value.get("y"),
                raw_value.get("yaw", 0.0),
            )
        elif isinstance(raw_value, (list, tuple)) and len(raw_value) >= 2:
            parts = (
                raw_value[0],
                raw_value[1],
                raw_value[2] if len(raw_value) >= 3 else 0.0,
            )
        else:
            raise ValueError(f"Waypoint {name} has no valid coordinate")
        try:
            waypoint = tuple(float(value) for value in parts)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"Waypoint {name} contains a non-numeric coordinate"
            ) from exc
        if not all(math.isfinite(value) for value in waypoint):
            raise ValueError(f"Waypoint {name} contains a non-finite value")
        normalized[name] = waypoint
    return dict(
        sorted(
            normalized.items(), key=lambda item: waypoint_sort_key(item[0])
        )
    )


def normalize_zone_name(value: Any) -> str:

    name = str(value or "").strip().upper()
    if ZONE_NAME_RE.fullmatch(name) is None:
        raise ValueError("Zone names must be one letter from A to Z")
    return name


def normalize_zones(values: Mapping[str, Any]) -> Dict[str, ZoneBounds]:

    normalized: Dict[str, ZoneBounds] = {}
    for raw_name, raw_value in values.items():
        name = normalize_zone_name(raw_name)
        if isinstance(raw_value, ZoneBounds):
            parts = raw_value.as_tuple()
        elif isinstance(raw_value, Mapping):
            parts = (
                raw_value.get("x_min"),
                raw_value.get("y_min"),
                raw_value.get("x_max"),
                raw_value.get("y_max"),
            )
        elif isinstance(raw_value, (list, tuple)) and len(raw_value) == 4:
            parts = tuple(raw_value)
        else:
            raise ValueError(f"Zone {name} has no valid rectangular bounds")
        try:
            x_1, y_1, x_2, y_2 = (float(value) for value in parts)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"Zone {name} contains a non-numeric bound"
            ) from exc
        if not all(math.isfinite(value) for value in (x_1, y_1, x_2, y_2)):
            raise ValueError(f"Zone {name} contains a non-finite bound")
        x_min, x_max = sorted((x_1, x_2))
        y_min, y_max = sorted((y_1, y_2))
        if math.isclose(x_min, x_max) or math.isclose(y_min, y_max):
            raise ValueError(
                f"Zone {name} must have a non-zero width and height"
            )
        normalized[name] = ZoneBounds(x_min, y_min, x_max, y_max)
    return dict(sorted(normalized.items()))


@dataclass(frozen=True)
class RosMapSpec:


    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    def pixel_to_world(
        self, pixel_x: float, pixel_y: float, image_height: int
    ) -> Tuple[float, float]:
        local_x = float(pixel_x) * self.resolution
        local_y = (float(image_height) - float(pixel_y)) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        world_x = self.origin_x + cosine * local_x - sine * local_y
        world_y = self.origin_y + sine * local_x + cosine * local_y
        return world_x, world_y

    def world_to_pixel(
        self, world_x: float, world_y: float, image_height: int
    ) -> Tuple[float, float]:
        delta_x = float(world_x) - self.origin_x
        delta_y = float(world_y) - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return (
            local_x / self.resolution,
            float(image_height) - local_y / self.resolution,
        )

    def contains_world(
        self,
        world_x: float,
        world_y: float,
        image_width: int,
        image_height: int,
    ) -> bool:
        pixel_x, pixel_y = self.world_to_pixel(
            world_x, world_y, image_height
        )
        return (
            0.0 <= pixel_x <= float(image_width)
            and 0.0 <= pixel_y <= float(image_height)
        )

    def world_bounds(
        self, image_width: int, image_height: int
    ) -> Tuple[float, float, float, float]:
        corners = [
            self.pixel_to_world(0, 0, image_height),
            self.pixel_to_world(image_width, 0, image_height),
            self.pixel_to_world(0, image_height, image_height),
            self.pixel_to_world(image_width, image_height, image_height),
        ]
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        return min(xs), max(xs), min(ys), max(ys)


def load_ros_map_yaml(path: str | Path) -> RosMapSpec:

    yaml_path = Path(path).expanduser().resolve()
    if not yaml_path.is_file():
        raise ValueError(f"Map YAML was not found: {yaml_path}")
    try:
        payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Map YAML could not be read: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Map YAML must contain a mapping")

    image_value = str(payload.get("image", "")).strip()
    if not image_value:
        raise ValueError("Map YAML does not define an image")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise ValueError(f"Map image was not found: {image_path}")

    try:
        resolution = float(payload["resolution"])
        origin = tuple(float(value) for value in payload["origin"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Map resolution or origin is invalid") from exc
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError("Map resolution must be a positive finite number")
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError("Map origin must contain finite x, y, and yaw")
    return RosMapSpec(
        yaml_path=yaml_path,
        image_path=image_path,
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
    )


def discover_ros_maps(directories: Iterable[str | Path]) -> List[Path]:

    discovered: Dict[str, Path] = {}
    for raw_directory in directories:
        directory = Path(raw_directory).expanduser()
        if not directory.is_dir():
            continue
        candidates = list(directory.glob("*.yaml"))
        candidates.extend(directory.glob("*.yml"))
        for candidate in candidates:
            if candidate.stem.endswith("_waypoints"):
                continue
            try:
                spec = load_ros_map_yaml(candidate)
            except ValueError:
                continue
            discovered[str(spec.yaml_path)] = spec.yaml_path
    return sorted(
        discovered.values(), key=lambda path: (path.stem.lower(), str(path))
    )


def waypoint_config_path(map_yaml_path: str | Path) -> Path:

    path = Path(map_yaml_path).expanduser().resolve()
    return path.with_name(f"{path.stem}_waypoints.yaml")


def load_map_annotations(path: str | Path) -> MapAnnotations:

    config_path = Path(path).expanduser().resolve()
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Waypoint file could not be read: {exc}") from exc
    if not isinstance(payload, dict) or not isinstance(
        payload.get("waypoints"), dict
    ):
        raise ValueError("Waypoint file must contain a waypoints mapping")
    zones_defined = "zones" in payload
    raw_zones = payload.get("zones", {})
    if not isinstance(raw_zones, dict):
        raise ValueError("Waypoint file zones must contain a mapping")
    return MapAnnotations(
        waypoints=normalize_waypoints(payload["waypoints"]),
        zones=normalize_zones(raw_zones),
        zones_defined=zones_defined,
    )


def load_waypoint_config(path: str | Path) -> Dict[str, Waypoint]:

    return load_map_annotations(path).waypoints


def save_waypoint_config(
    path: str | Path,
    map_yaml_path: str | Path,
    waypoints: Mapping[str, Any],
    zones: Mapping[str, Any] | None = None,
) -> Path:

    config_path = Path(path).expanduser().resolve()
    normalized = normalize_waypoints(waypoints)
    if zones is None and config_path.is_file():
        try:
            existing = load_map_annotations(config_path)
            normalized_zones = existing.zones
            zones_defined = existing.zones_defined
        except ValueError:
            normalized_zones = {}
            zones_defined = False
    else:
        normalized_zones = normalize_zones(zones or {})
        zones_defined = zones is not None
    payload = {
        "map": str(Path(map_yaml_path).expanduser().resolve()),
        "frame_id": "map",
        "waypoints": {
            name: {
                "x": round(value[0], 4),
                "y": round(value[1], 4),
                "yaw": round(value[2], 6),
            }
            for name, value in normalized.items()
        },
    }
    if zones_defined:
        payload["zones"] = {
            name: {
                "x_min": round(bounds.x_min, 4),
                "y_min": round(bounds.y_min, 4),
                "x_max": round(bounds.x_max, 4),
                "y_max": round(bounds.y_max, 4),
            }
            for name, bounds in normalized_zones.items()
        }
    config_path.parent.mkdir(parents=True, exist_ok=True)
    content = yaml.safe_dump(payload, sort_keys=False, allow_unicode=False)
    config_path.write_text(content, encoding="utf-8")
    return config_path
