from pathlib import Path

from my_robot_package.calibration_check import calibration_findings


def test_snapshot_calibration_is_consistent_but_incomplete():
    config_directory = Path(__file__).parents[1] / "config"
    errors, incomplete = calibration_findings(config_directory)
    assert errors == []
    assert "wheel_geometry_measured" in incomplete
    assert "oakd_mount_transform_measured" in incomplete
