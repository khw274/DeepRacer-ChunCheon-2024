# 2024년 AWS DeepRacer Championship 춘천
제2회 AWS DeepRacer Championship 춘천

```결승 진출```

## 공모 내용
 AWS DeepRacer를 활용한 인공지능 강화학습 기반의 자율주행 자동차 AI 융합 경진대회

## 대회 진행
### 트랙
#### 예선: Smile Speedway(Counterclockwise)  
![image](https://github.com/user-attachments/assets/d161e888-154e-4c00-9ed1-1a01d58de8a8)

#### 본선: 2022 re:Invent Championship(Counterclockwise)
![image](https://github.com/user-attachments/assets/3c56e19b-9998-4c72-afe8-24c2dc911885)

### 규칙
- 1바퀴 완주 기록 기준 (트랙 이탈 시 3초 패널티)   
- 모델 훈련시간 : 30시간  
- 모델 제출 제한 횟수 : 50회

### 예선
작년에 참가했던 2023년 인천 딥레이서 챔피언십에서 사용한 트랙 Smile Speedway와 같은 트랙이라 좀 더 수월할 것이라 예상하고 예선전에 임하였다.

하지만 예상과 달리 같은 코드와 파라미터, 조향각을 사용했음에도 좋은 랩타임이 나오지 않아 별도의 수정 과정을 거쳤다.

변경사항은 총 3가지로 코드 최적화, 코드 파라미터 수정, 조향각과 속도 수정이다.

#### 코드 최적화) 
이전에 사용하던 코드는 여러 테스트들이 묶여 있어 현재 AWS 시스템에서 사용이 안되는 것을 확인하였다.

테스트 하는 부분은 실제 작동하는데 영향을 끼치지 않아 모두 삭제하였고 더 직관적인 코드를 구성할 수 있었다.

#### 코드 파라미터 수정)
훈련에 사용하는 보상함수 코드에는 다음과 같은 주요 세가지 파라미터 수정 포인트가 있다.

#### 1. get_target_point 함수의 r 값:

설정한 값: r = params['track_width'] * 0.45  
파라미터 설명: 이 반지름(r)은 차량의 위치를 기준으로 웨이포인트가 "안쪽" 또는 "바깥쪽"에 있는지 판단하는 데 사용된다. 이 값을 조정하면 차량이 트랙의 중앙선을 얼마나 가깝게 따라야 하는지를 변경할 수 있다.  

수정 포인트: 현재 0.45로 설정된 배수를 늘리거나 줄이면 트랙을 따르는 "밀도"가 바뀌게 된다. 더 큰 값은 차량이 트랙의 중앙을 더 가깝게 따르도록 하고, 더 작은 값은 차량이 더 느슨하게 트랙을 따르도록 할 수 있다.  

#### 2. up_sample 함수의 factor 값:

설정한 값: up_sample(waypoints, 20)  
설명: 이 factor는 추가한 최적좌표 웨이포인트 사이사이에 얼마나 더 많은 웨이포인트들을 추가할지를 제어한다. 더 높은 값은 기존 웨이포인트 사이에 더 많은 웨이포인트를 추가하여 경로 계획을 부드럽게 하지만 계산 부하가 증가한다.  

수정 포인트: 이 값을 조정하면 웨이포인트의 밀도가 변경된다. 더 높은 값은 차량이 경로를 더 부드럽게 따라가게 만들 수 있지만 계산 속도가 느려질 수 있다. 반대로 더 낮은 값은 덜 정밀하지만 계산이 더 빨라질 수 있다.  

#### 3. score_steer_to_point_ahead 함수의 오류 스케일링:

현재 수식: error = (steering_angle - best_steering_angle) / 60.0  
설명: 이 줄은 현재 조향 각도와 최적 조향 각도 간의 오차를 계산하며, 60.0으로 나누어 스케일링한다. 이 스케일링 값은 원하는 조향 각도에 대한 오류를 얼마나 크게 패널티로 처리할지를 결정한다.  

수정 포인트: 60.0을 다른 값으로 변경하면 조향 오류에 대한 보상 민감도가 조정된다. 더 작은 값을 사용하면 함수가 조향 오류에 대해 더 민감하게 반응하여 더 정확한 조향을 유도할 수 있고, 더 큰 값은 오류에 대해 더 관대하게 반응하게 됩니다.
