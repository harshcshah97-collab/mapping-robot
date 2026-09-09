"""Persistent, validated semantic room definitions for the robot app."""

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
import re


ROOM_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,39}$")


def normalize_room_name(name):
    """Return a case-insensitive key for a human-readable room name."""
    return " ".join(str(name).strip().casefold().split())


def polygon_centroid(points):
    """Return a stable centroid for a polygon, with a mean fallback."""
    signed_area = 0.0
    centroid_x = 0.0
    centroid_y = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        cross = point["x"] * next_point["y"] - next_point["x"] * point["y"]
        signed_area += cross
        centroid_x += (point["x"] + next_point["x"]) * cross
        centroid_y += (point["y"] + next_point["y"]) * cross
    signed_area *= 0.5
    if abs(signed_area) < 1e-9:
        return {
            "x": sum(point["x"] for point in points) / len(points),
            "y": sum(point["y"] for point in points) / len(points),
        }
    scale = 1.0 / (6.0 * signed_area)
    return {"x": centroid_x * scale, "y": centroid_y * scale}


def point_in_polygon(point, polygon):
    """Return whether a point is inside or on the edge of a polygon."""
    x_value = float(point["x"])
    y_value = float(point["y"])
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        x1, y1 = float(previous["x"]), float(previous["y"])
        x2, y2 = float(current["x"]), float(current["y"])
        cross = (x_value - x1) * (y2 - y1) - (y_value - y1) * (x2 - x1)
        if abs(cross) < 1e-7:
            if min(x1, x2) - 1e-7 <= x_value <= max(x1, x2) + 1e-7:
                if min(y1, y2) - 1e-7 <= y_value <= max(y1, y2) + 1e-7:
                    return True
        crosses = (y1 > y_value) != (y2 > y_value)
        if crosses:
            intersection_x = (x2 - x1) * (y_value - y1) / (y2 - y1) + x1
            if x_value < intersection_x:
                inside = not inside
    return inside


def _cross_product(first, second, third):
    return (
        (second["x"] - first["x"]) * (third["y"] - first["y"])
        - (second["y"] - first["y"]) * (third["x"] - first["x"])
    )


def _segments_intersect(first, second, third, fourth):
    """Return whether two non-adjacent polygon edges intersect."""
    products = (
        _cross_product(first, second, third),
        _cross_product(first, second, fourth),
        _cross_product(third, fourth, first),
        _cross_product(third, fourth, second),
    )
    return products[0] * products[1] <= 0 and products[2] * products[3] <= 0


def _validate_polygon_shape(polygon):
    unique_points = {(point["x"], point["y"]) for point in polygon}
    if len(unique_points) != len(polygon):
        raise ValueError("A room boundary cannot contain duplicate points.")
    edge_count = len(polygon)
    for first_index in range(edge_count):
        second_index = (first_index + 1) % edge_count
        for third_index in range(first_index + 1, edge_count):
            fourth_index = (third_index + 1) % edge_count
            if len({first_index, second_index, third_index, fourth_index}) < 4:
                continue
            if _segments_intersect(
                polygon[first_index],
                polygon[second_index],
                polygon[third_index],
                polygon[fourth_index],
            ):
                raise ValueError("The room boundary cannot cross itself.")
    twice_area = abs(sum(
        point["x"] * polygon[(index + 1) % len(polygon)]["y"]
        - polygon[(index + 1) % len(polygon)]["x"] * point["y"]
        for index, point in enumerate(polygon)
    ))
    if twice_area < 0.02:
        raise ValueError("The room boundary is too small or forms a line.")


def validate_room(payload, available_maps):
    """Validate and normalize one room payload from the app."""
    if not isinstance(payload, dict):
        raise ValueError("Room data must be an object.")
    name = " ".join(str(payload.get("name", "")).strip().split())
    if not ROOM_NAME_PATTERN.fullmatch(name):
        raise ValueError("Use a 1-40 character room name with letters or numbers.")
    map_name = Path(str(payload.get("map", ""))).name
    if map_name not in available_maps:
        raise ValueError("Select an available map before creating a room.")
    raw_polygon = payload.get("polygon")
    if not isinstance(raw_polygon, list) or not 3 <= len(raw_polygon) <= 32:
        raise ValueError("A room needs between 3 and 32 map points.")
    polygon = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, dict):
            raise ValueError("Every room point must contain x and y coordinates.")
        try:
            x_value = float(raw_point["x"])
            y_value = float(raw_point["y"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("Room coordinates must be numbers.") from error
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            raise ValueError("Room coordinates must be finite.")
        if abs(x_value) > 100.0 or abs(y_value) > 100.0:
            raise ValueError("Room coordinates exceed the supported map size.")
        polygon.append({"x": round(x_value, 4), "y": round(y_value, 4)})

    _validate_polygon_shape(polygon)

    centroid = polygon_centroid(polygon)
    raw_goal = payload.get("goal") or centroid
    try:
        goal = {
            "x": float(raw_goal["x"]),
            "y": float(raw_goal["y"]),
            "yaw": float(raw_goal.get("yaw", 0.0)),
        }
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("The room goal must contain numeric x, y, and yaw.") from error
    if not all(math.isfinite(value) for value in goal.values()):
        raise ValueError("The room goal must be finite.")
    if not point_in_polygon(goal, polygon):
        raise ValueError("The room navigation goal must be inside its boundary.")

    return {
        "name": name,
        "key": normalize_room_name(name),
        "map": map_name,
        "polygon": polygon,
        "goal": {key: round(value, 4) for key, value in goal.items()},
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }


class RoomStore:
    """Atomic JSON-backed room storage shared by app and voice control."""

    def __init__(self, path):
        self.path = Path(path).expanduser()

    def _read(self):
        try:
            payload = json.loads(self.path.read_text())
        except FileNotFoundError:
            return {"version": 1, "rooms": []}
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError("Room database is unreadable: %s" % error) from error
        if payload.get("version") != 1 or not isinstance(payload.get("rooms"), list):
            raise RuntimeError("Room database format is unsupported.")
        return payload

    def _write(self, payload):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        temporary.chmod(0o600)
        os.replace(temporary, self.path)

    def list(self, map_name=None):
        """List rooms, optionally filtering by map file name."""
        rooms = self._read()["rooms"]
        if map_name:
            safe_map = Path(str(map_name)).name
            rooms = [room for room in rooms if room["map"] == safe_map]
        return sorted(rooms, key=lambda room: room["name"].casefold())

    def upsert(self, payload, available_maps):
        """Create or replace a room with the same map and normalized name."""
        room = validate_room(payload, available_maps)
        database = self._read()
        database["rooms"] = [
            existing for existing in database["rooms"]
            if not (
                existing["map"] == room["map"]
                and existing["key"] == room["key"]
            )
        ]
        database["rooms"].append(room)
        self._write(database)
        return room

    def delete(self, map_name, room_name):
        """Delete one room, returning whether it existed."""
        database = self._read()
        key = normalize_room_name(room_name)
        safe_map = Path(str(map_name)).name
        retained = [
            room for room in database["rooms"]
            if not (room["map"] == safe_map and room["key"] == key)
        ]
        changed = len(retained) != len(database["rooms"])
        if changed:
            database["rooms"] = retained
            self._write(database)
        return changed

    def resolve(self, room_name, map_name=None):
        """Resolve a spoken room name, rejecting ambiguous cross-map matches."""
        key = normalize_room_name(room_name)
        matches = [room for room in self.list(map_name) if room["key"] == key]
        if not matches:
            raise KeyError("Unknown room: %s" % room_name)
        if len(matches) > 1:
            raise ValueError("Room name exists on multiple maps; choose a map.")
        return matches[0]
