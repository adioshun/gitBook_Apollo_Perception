# [Apollo Perceptual Analysis Tracking Object Information Fusion](https://mp.weixin.qq.com/s?__biz=MzI1NjkxOTMyNQ==&mid=2247485916&idx=1&sn=5510b30bde81a10c0271e526151a8cac&scene=19#wechat_redirect)

  

  

  

아폴로 퓨전 모델은 두가지 구성 요소로 이루어져 있다. 관성 네비게션 + 칼만 필터 `In the fusion framework of the Apollo multi-sensor fusion positioning module,Including two parts: inertial navigation solution, Kalman filter;`

  

The results of the fusion positioning will in turn be used for the prediction of GNSS positioning and point cloud positioning;

  

퓨전 위치의 결과물은 다음과 같다. `The output of the fusion positioning is a 6-dof position and pose, and a covariance matrix.`

- 6 DOF Position 
- Pose
- covariance matrix

아폴로는 헝가리언 알고리즘을 이용하여서 매 수간 탐지된 물체를 **추적 리스트 **에서 매칭시킨다. `In the previous step of the **HM object tracking** step, Apollo performs a binary map matching of the Hungarian algorithm between the Object detected at each moment and the TrackedObject in the tracking list. `

The matching results are divided into three categories:
1.  기존 차량이면, 칼만필터를 사용하여 상태를 업데이트 한다. `If the match is successful, the Kalman filter is used to update the information (center of gravity, speed, acceleration) and other information;`
    
2.  없던 차량이면, 추적 리스트에 추가 한다. ` If there is a mismatch, the corresponding TrackedObject is missing, the Object is encapsulated into a TrackedObject, and the tracking list is added;`
    
3.  추적 실패 차량은 최근 탐지 시간을 사용하여 오래된 것이면 추적버퍼에서 제거 한다. `For trackedObjects whose target is missing at the current time in the tracking list (Object cannot match), use the time of the last time, the position of the center of gravity and the acceleration of the new current time (cannot be updated with Kalman filter, lack of observation state). For those lost tracking targets that are too long, remove them from the tracking queue.`


