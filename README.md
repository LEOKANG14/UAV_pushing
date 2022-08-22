## 20220822
현재 구조에서,
1. is_last_try가 False 일때 (parcel handover없을때)는 무조건 UAV는 한번만 배송하고 트럭으로 가게하고//
is_last_try가 True 일때는 UAV_path가 되도록이면 먼저 DR_node에 붙여서 주르륵 이어지게끔 고쳐야 될 것 같습니다. (트럭path의 dpt-arv 시간 차이를 비교해서 UAV_path를 새로 파는 것이 유리하면 새로 생성)

2. 트럭 path의 dpt-arv 시간 차이는 똑같을 것이라서 비교가 무의미할것 같다고 하셨는데, UAV_path가 새로 만들어지면 트럭 경로 사이에 +30sec이 되기 때문에 target node는 그냥 기존에 존재하던 UAV_path에 붙으려고 할 것입니다.
이를 위해서, SK가 출발하고 도착할 때 트럭 accum time에 +30sec 씩 해주는 메쏘드가 또 필요할 것 같습니다

3. 위에 두개 이슈 해결되면 refactoring 한번 하면 될 것 같습니다


# 우선순위
1. is_last_try == False
- DR에도 붙여보고 UAV에도 붙여보고 비교해서 좋은거 선택(w/o parcel handover)
2. is_last_try == True
- DR에도 붙여보고 UAV(w/t parcel handover)로 붙여서 비교해서 좋은거 선택
3. 2를 before/after 가 변하지 않을 때 까지 해보기
4. is_last_try == True
- DR에도 붙여보고 UAV(w/o parcel handover)로 붙여서 비교해서 좋은거 선택

# TODO
- get_arv_node()에서 SK path에 loading time 더할 때 dpt node, dr node(if parcel handover)에서만 loading time 더해지도록 수정
- get_truck_wait_time()을 start, end node 받아서 특정 range에 대해서 구하도록 수정
- update시 min time 구하는 곳에 get_truck_wait_time(0,-1) time 더하도록 수정

- mode == 'default'일 때, parcel handover 안하도록 수정
- mode == 'parcel handover'일 때, parcel handover

- input list에 대해 default로 try
- input list에 대해 parcel handover로 try
  - fail node list에 대해 parcel handover로 try
  - fail node list의 before/after 가 같으면
  - default mode로 try
  - 종료