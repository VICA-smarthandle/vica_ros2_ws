# vica_destination_manager

Flutter 관리자 앱의 목적지 JSON 요청을 지도별 YAML로 저장하는 ROS 2 패키지다.

```text
/save_location, /delete_location_request, /location_list_request
→ vica_destination_manager
→ ~/vica_data/destinations/<map_id>/destinations.yaml
→ /location_list
```

- 기존 `locations.json`은 읽거나 이관하지 않는다.
- 목적지 ID는 canonical UUID v4만 허용한다.
- `contact_phone`(물류 배송 도착 문자 연락처)은 숫자만 남겨 저장하고 휴대폰 번호 모양이 아니면 거부한다. 로봇은 읽지 않는다.
- 저장 경로는 요청 JSON이 아니라 `storage_root` ROS 파라미터로 관리한다.
- 저장·삭제는 임시 파일을 같은 디렉터리에 쓴 후 교체한다.
- 변경 뒤 `/vica/mission/reload_destinations`를 호출한다.

```bash
ros2 launch vica_destination_manager destination_manager.launch.py
```
