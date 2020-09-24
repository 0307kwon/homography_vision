# homography_vision
단안카메라 2대의 상대 위치를 추정하기 위한 패키지 (현재 미완성)

## 1.summary
최종 목표는 드론 2대의 하부에 각각 단안카메라를 설치하고
이를 통해 받아오는 이미지를 feature matching 과 homography 추정을 통해
드론 2대의 상대 position과 orientation을 출력하는 것입니다.

<img src="./image/image0.png" width="50%"></img>

## 2.how to use (zed_mini 사용 시)
현재 구조는 zed_mini 한 대로 2대의 역할을 하도록 되어있습니다. 

image_in1 : zed_mini의 image_rect_color영상을 사진으로 저장하여 불러옴

image_in2 : zed_mini의 /zedm/zed_node/rgb/image_rect_color 토픽에서 실시간으로 불러옴


### 1. homography_vision/src/main.cpp 를 수정해야합니다.
  #### 1-1.
  31번째 줄 수정
  ``` cpp
  double altitude = (제드의 높이를 m단위로);
  ```
  #### 1-2.
  64번째 줄 수정
  ``` cpp
    image_in1 = imread("원하는 이미지가 있는 path",IMREAD_COLOR);
  ```
  #### + zed_mini를 쓰지 않을 시
  83,84번째 줄 수정( 현재 imu데이터와 이미지는 zed_mini에서 받아오는 것으로 되어있음 ) 
  ``` cpp
  message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh,"이미지 토픽",10);
  message_filters::Subscriber<sensor_msgs::Imu> rpy2_sub(nh,"imu 토픽",10);
  ```
  => zed_mini를 쓰지 않을 시 다른 메세지를 subscribe 하도록 수정.
  
### 2. roslaunch zed_wrapper zedm.launch
  => zed_mini를 실행합니다. 
### 3. rosrun homography_vision homography_vision
  => 본 패키지 실행

## 3. 실행 결과
<img src="./image/중간 위치 - 성공(이동전).png" width="35%"></img>
<img src="./image/중간 위치 - 성공(이동후).png" width="35%"></img>
<img src="./image/낮은 위치 - 성공(이동전).png" width="35%"></img>
<img src="./image/낮은 위치 - 성공(이동후).png" width="35%"></img>
