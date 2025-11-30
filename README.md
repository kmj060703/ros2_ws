### 한 번에 다 실행시키는 방법
```bash
ros2 launch master_jo total.launch.py
```
### turtlebot 카메라 실행 방법 
```bash
cd ros2_ws && colcon build
source install/setup.bash && ros2 launch usb_cam camera.launch.py
```

### turtlebot_vision 실행 방법
#### sudo apt install python3-pip
#### pip uninstall -y opencv-python opencv-contrib-python numpy
#### pip install numpy==1.23.5
#### pip install opencv-python==4.6.0.66
#### pip install -U ultralytics
#### turtlebot_vision.py에서 절대경로 설정 바꿔주기
```bash
ros2 run turtlebot_vision vision_subscriber
```

### vision_hyun 실행 방법
```bash
ros2 run vision_hyun vision_hyun_node 
```

### imu_jo 실행 방법
```bash
ros2 run master_jo imu_jo
```

### psd_jo 실행 방법
```bash
ros2 run master_jo psd_jo
```

### driving_yy 실행 방법
```bash
ros2 run driving_yy driving_node
```

### ui_test 실행 방법
```bash
ros2 run ui_test ui_test
```
#### ui에서는 Home탭이랑 PD탭 동시에 제어 못함 
#### Home탭에서는 사용자 조종
#### PD 탭에서는 pd제어 들어가고 vision, master_jo, driving_yy 켜져있어야 작동함
