#!/usr/bin/env python3
"""Flutter 목적지 요청을 지도별 destinations.yaml과 동기화한다."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .storage import DestinationStorage, validate_map_id


class DestinationManagerNode(Node):
    """목적지 저장·조회·삭제와 Mission Manager reload를 담당한다."""

    def __init__(self) -> None:
        super().__init__("vica_destination_manager")
        self.declare_parameter(
            "storage_root",
            str(Path.home() / "vica_data" / "destinations"),
        )
        self.declare_parameter(
            "mission_reload_service",
            "/vica/mission/reload_destinations",
        )

        root = str(self.get_parameter("storage_root").value)
        self.storage = DestinationStorage(root)
        self.location_publisher = self.create_publisher(String, "/location_list", 10)
        self.create_subscription(String, "/save_location", self._on_save, 10)
        self.create_subscription(
            String,
            "/location_list_request",
            self._on_list_request,
            10,
        )
        self.create_subscription(
            String,
            "/delete_location_request",
            self._on_delete,
            10,
        )

        reload_service = str(self.get_parameter("mission_reload_service").value)
        self.reload_client = self.create_client(Trigger, reload_service)
        self.get_logger().info(
            f"vica_destination_manager 시작: storage_root={self.storage.root}"
        )

    def _on_save(self, msg: String) -> None:
        try:
            payload = self._decode(msg)
            map_id = validate_map_id(payload.get("map_id", ""))
            destination = {
                key: value
                for key, value in payload.items()
                if key not in {"request_id", "map_id", "timestamp", "storage_root"}
            }
            destinations = self.storage.upsert(map_id, destination)
        except (ValueError, TypeError, json.JSONDecodeError, OSError) as exc:
            self.get_logger().error(f"목적지 저장 거부: {exc}")
            return
        self._publish(map_id, destinations)
        self._reload_mission_catalog("저장", map_id)
        self.get_logger().info(
            f"목적지 저장 완료: map_id={map_id} id={destination.get('id')}"
        )

    def _on_list_request(self, msg: String) -> None:
        try:
            payload = self._decode(msg)
            map_id = validate_map_id(payload.get("map_id", ""))
            destinations = self.storage.read(map_id)
        except (ValueError, TypeError, json.JSONDecodeError, OSError) as exc:
            self.get_logger().error(f"목적지 목록 요청 거부: {exc}")
            return
        self._publish(map_id, destinations)

    def _on_delete(self, msg: String) -> None:
        try:
            payload = self._decode(msg)
            map_id = validate_map_id(payload.get("map_id", ""))
            destination_id = str(
                payload.get("destination_id") or payload.get("location_id") or ""
            )
            destinations = self.storage.delete(map_id, destination_id)
        except (ValueError, TypeError, json.JSONDecodeError, OSError) as exc:
            self.get_logger().error(f"목적지 삭제 거부: {exc}")
            return
        self._publish(map_id, destinations)
        self._reload_mission_catalog("삭제", map_id)
        self.get_logger().info(
            f"목적지 삭제 완료: map_id={map_id} id={destination_id}"
        )

    @staticmethod
    def _decode(msg: String) -> dict[str, Any]:
        payload = json.loads(msg.data)
        if not isinstance(payload, dict):
            raise ValueError("요청 JSON은 객체여야 합니다")
        return payload

    def _publish(self, map_id: str, destinations: list[dict[str, Any]]) -> None:
        msg = String()
        msg.data = json.dumps(
            {"map_id": map_id, "locations": destinations},
            ensure_ascii=False,
        )
        self.location_publisher.publish(msg)

    def _reload_mission_catalog(self, operation: str, map_id: str) -> None:
        if not self.reload_client.service_is_ready():
            self.get_logger().warn(
                f"목적지 {operation}은 완료됐지만 Mission reload service가 없습니다: "
                f"map_id={map_id}"
            )
            return
        future = self.reload_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._on_reload_result(result, operation, map_id)
        )

    def _on_reload_result(self, future: Any, operation: str, map_id: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"목적지 {operation} 후 Mission reload 실패: map_id={map_id} reason={exc}"
            )
            return
        if response is None or not response.success:
            reason = "응답 없음" if response is None else response.message
            self.get_logger().error(
                f"목적지 {operation} 후 Mission reload 거부: "
                f"map_id={map_id} reason={reason}"
            )
            return
        self.get_logger().info(
            f"목적지 {operation} 후 Mission catalog reload 완료: map_id={map_id}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DestinationManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
