# fallback_rotation

모든 후보가 reject될 때 좌우 장애물 개수로 회전 방향을 정해 제자리 회전.
Real은 Sim과 달리 지속 lock 상태를 두지 않고 매 순간 반응한다.

호출: [[compute_command]] · 비교: [[go2_dwa_real]]
