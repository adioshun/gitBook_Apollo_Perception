# Apollo Perception 

바이두의 자율 주행 차량 플랫폼인 Apollo의 인지(Perception)부분을 정리한 페이지 입니다. 개인 연구 노트로 가독성 중심으로 분류 및 간략한 요점만 작성되어 있습니다. 


---

Apollo의 인지 모듈은 크게 2개로 나누어 집니다. 

- 1. Obstacle detection : LIDAR/RADAR 3D 분석 

- 2. Traffic light perception : 2D 이미지 분석

Obstacles Perception : Lidar sensing module의 기능은 다음과 같습니다. 

- 1.1 High-precision map ROI filter : HD맵을 이용한 도로와 도로변(나무, 건물 등) 구분 

- 1.2 Segmentation based on convolutional neural networks : CNN(U-Net)기반 물체 세그멘테이션 

- 1.3 MinBox obstacle frame construction : 자세 추정, 진행 방향을 알기 위한 bbox 생성 

- 1.4 HM object tracking : 물체 추적 

[1.2]의 Segmentation based on CNN은 아래 4가지 모듈로 구성 됩니다. 

1.2-A : Channel feature extraction

1.2-B : Obstacle prediction based on convolutional neural network

1.2-C : Obstacle cluster

1.2-D : Post processing

[1.2-A]에서는 8개의 Feature를 추출 합니다.  

. The maximum height of the point in the cell

. The strength of the highest point in the cell

. The average height of the points in the cell

. Average intensity of the points in the cell

. Number of points in the cell

. The angle of the cell center relative to the origin

. The distance between the center of the cell and the origin

. The binary value indicates whether the cell is empty or occupied (is there a bit in the cell)

[1.2-B]에서는 위 8개의 특징을 CNN에 입력 하여 아래 4개의 정보를 추출 합니다. 

. Objectness : 탐지 목적 

. Center offset : 군집화 목적 

. Positiveness : 필터링 목적 

. Object height : 필터링 목적 

. Class probability : 분류 목적 
