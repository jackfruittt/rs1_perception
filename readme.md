## To Test
Make sure you have apriltag_detector stuff for the plugins and etc.

sudo apt install ros-humble-apriltag-detector ros-humble-apriltag-detector-mit ros-humble-apriltag-detector-umich

## To Run
Build package in your ws then execute

ros2 launch rs1_perception perception.launch.py # This only works for just 1 drone for now (/rs1_drone_1/front/image)

## Topics to Echo

ros2 topic echo /topic_name

/tags # From apriltag_detector, it is an apriltag detections array that has the tag id and other data

/scenario_detection # Our publisher that publishes a string message that represents scenario, drone position and other data

