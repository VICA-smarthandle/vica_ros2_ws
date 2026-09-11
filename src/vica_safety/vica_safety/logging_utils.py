"""ROS 2 Humble-compatible logging helpers."""


def log_with_severity(logger, severity: str, message: str) -> None:
    """Log changing severities from distinct call sites."""
    if severity == "error":
        logger.error(message)
    elif severity == "warn":
        logger.warning(message)
    elif severity == "info":
        logger.info(message)
    else:
        raise ValueError(f"unsupported log severity: {severity}")
