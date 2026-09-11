"""Host Nav2가 nvblox costmap plugin을 로드할 수 있는지 검증하는 헬퍼."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)

NVBLOX_PLUGIN_PACKAGE = "nvblox_nav2"
NVBLOX_PLUGIN_XML = "nvblox_costmap_layer.xml"
NVBLOX_LAYER_CLASS = "nvblox::nav2::NvbloxCostmapLayer"
NVBLOX_PLUGIN_LIBRARY = "libnvblox_nav2.so"
NVBLOX_MSGS_PACKAGE = "nvblox_msgs"
NVBLOX_SLICE_MSG = "DistanceMapSlice.msg"


class NvbloxPluginError(RuntimeError):
    """nvblox costmap plugin 리소스가 깨져 로드 불가할 때."""


def nvblox_packages_present() -> bool:
    """nvblox_nav2 / nvblox_msgs가 ament index에서 발견되는지."""
    for package in (NVBLOX_PLUGIN_PACKAGE, NVBLOX_MSGS_PACKAGE):
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            return False
    return True


def plugin_xml_path() -> Path:
    share = Path(get_package_share_directory(NVBLOX_PLUGIN_PACKAGE))
    return share / NVBLOX_PLUGIN_XML


def plugin_library_path() -> Path:
    prefix = Path(get_package_prefix(NVBLOX_PLUGIN_PACKAGE))
    return prefix / "lib" / NVBLOX_PLUGIN_LIBRARY


def slice_msg_path() -> Path:
    share = Path(get_package_share_directory(NVBLOX_MSGS_PACKAGE))
    return share / "msg" / NVBLOX_SLICE_MSG


def verify_nvblox_costmap_plugin() -> None:
    """nvblox_layer가 실제로 로드 가능한 상태인지 검증. 문제 시 예외."""
    xml_path = plugin_xml_path()
    if not xml_path.is_file():
        raise NvbloxPluginError(
            f"plugin description가 없다(dangling symlink 가능): {xml_path}"
        )
    text = xml_path.read_text(encoding="utf-8").strip()
    if not text:
        raise NvbloxPluginError(
            f"plugin description가 비어 있다(dangling symlink): {xml_path}"
        )
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise NvbloxPluginError(f"plugin description 파싱 실패: {xml_path}: {exc}")

    classes = {element.get("type") for element in root.iter("class")}
    if NVBLOX_LAYER_CLASS not in classes:
        raise NvbloxPluginError(
            f"{NVBLOX_LAYER_CLASS} 선언이 없다: {xml_path} (declared={sorted(classes)})"
        )

    library = plugin_library_path()
    if not library.resolve().is_file():
        raise NvbloxPluginError(
            f"plugin library가 실제 파일로 resolve되지 않는다: {library}"
        )

    slice_msg = slice_msg_path()
    if not slice_msg.is_file():
        raise NvbloxPluginError(
            f"{NVBLOX_SLICE_MSG} 인터페이스가 없다: {slice_msg}"
        )
