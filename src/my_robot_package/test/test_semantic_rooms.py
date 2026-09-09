"""Tests for persistent semantic room definitions."""

import json

import pytest

from web_ui.semantic_rooms import RoomStore, point_in_polygon, validate_room


MAPS = {"home.yaml", "office.yaml"}


def room_payload(name="Kitchen"):
    """Return a square room with a valid central goal."""
    return {
        "name": name,
        "map": "home.yaml",
        "polygon": [
            {"x": 0, "y": 0},
            {"x": 2, "y": 0},
            {"x": 2, "y": 2},
            {"x": 0, "y": 2},
        ],
        "goal": {"x": 1, "y": 1, "yaw": 1.57},
    }


def test_room_validation_and_point_containment():
    room = validate_room(room_payload(), MAPS)
    assert room["key"] == "kitchen"
    assert point_in_polygon(room["goal"], room["polygon"])


def test_goal_outside_room_is_rejected():
    payload = room_payload()
    payload["goal"] = {"x": 4, "y": 4, "yaw": 0}
    with pytest.raises(ValueError, match="inside"):
        validate_room(payload, MAPS)


def test_self_crossing_room_is_rejected():
    payload = room_payload()
    payload["polygon"] = [
        {"x": 0, "y": 0},
        {"x": 2, "y": 2},
        {"x": 0, "y": 2},
        {"x": 2, "y": 0},
    ]
    with pytest.raises(ValueError, match="cross"):
        validate_room(payload, MAPS)


def test_store_upserts_resolves_and_deletes_atomically(tmp_path):
    path = tmp_path / "rooms.json"
    store = RoomStore(path)
    store.upsert(room_payload(), MAPS)
    replacement = room_payload("  KITCHEN ")
    replacement["goal"]["yaw"] = 0.25
    store.upsert(replacement, MAPS)

    assert len(store.list("home.yaml")) == 1
    assert store.resolve("kitchen")["goal"]["yaw"] == 0.25
    assert json.loads(path.read_text())["version"] == 1
    assert store.delete("home.yaml", "Kitchen")
    assert not store.list()
