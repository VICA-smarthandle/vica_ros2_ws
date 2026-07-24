"""Regression tests for ROS 2 Humble logger severity transitions."""

from rclpy.impl.rcutils_logger import RcutilsLogger

from vica_safety.logging_utils import log_with_severity


def test_logger_accepts_severity_transitions_from_same_helper():
    """Changing state severity must not terminate a Safety node."""
    logger = RcutilsLogger(name="vica_safety_logging_test")

    log_with_severity(logger, "error", "error state")
    log_with_severity(logger, "warn", "warning state")
    log_with_severity(logger, "info", "ready state")
