# 질문 기록 — 미션이 대답을 기다리는 말을 낼 때마다 낸다 (설계 2026-11-14 4.1, 미션 → 음성).
# 토픽 /vica/question, RELIABLE · TRANSIENT_LOCAL · depth 1. 질문이 닫히면 id 가 빈 기록을 낸다.
string id                        # 질문마다 새 값. "" = 열린 질문 없음(닫힘 알림)
string kind                      # open / confirm / choice / yesno / yesno_inverted / minutes
string text                      # 로봇이 말한 문장 그대로
string[] option_ids              # 고르기·확인이 묻는 목적지 id
string[] option_names            # 위와 같은 순서의 목적지 이름
uint8 step                       # 되묻기 단계 0~3 (되묻기 구현 전에는 항상 0)
string on_silence                # 끝까지 대답이 없을 때의 기본 동작 이름 (되묻기 구현 전에는 "")
builtin_interfaces/Time opened_at  # 질문을 낸 시각
