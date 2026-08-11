# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


# D212와 D213은 상호 배타적이다. D212는 요약을 첫 줄에, D213은 둘째 줄에 두라고 한다.
# 이 저장소는 D212 스타일(첫 줄 요약)을 쓰므로 D213을 명시적으로 뺀다.
#
#     """Return the knob-derived speed ratio, or 0.0 when any input is stale.
#
#     cmd 또는 knob 중 하나만 stale이어도 0.0(정지)을 반환한다.
#     """
#
# 근거: 이 저장소의 모든 패키지가 이 형태를 쓴다. ament 기본 convention은 D213을
# 검사하므로 빼지 않으면 저장소 전체가 이 검사를 통과하지 못한다.
# 이 판단의 정본은 vica_system_monitor/test/test_pep257.py 다(2026-07-31).
_EXTRA_IGNORE = 'D213'


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--add-ignore', _EXTRA_IGNORE])
    assert rc == 0, 'Found code style errors / warnings'
