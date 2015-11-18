# corobot_face_recognition

This ROS package performs face recognition on a live video stream using OpenCV. It follows the [ROS Action Protocol](http://wiki.ros.org/actionlib).

## Installation

1. [Install OpenCV 2.4.11](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html).
2. Install the [usb_cam](https://github.com/bosch-ros-pkg/usb_cam) package (Optional).
3. Install corobot_face_recognition.
```
cd ~/catkin_ws/src/
git clone https://github.com/ronitgalani/corobot_face_recognition
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## API

### Subscribed Topics
- `/usb_cam/image_raw` - videostream
- `/face_rec_order` - order and arguments for face recognition

### Published Topics
- `/corobot_face_recognition/feedback`
- `/corobot_face_recognition/result`

### Messages
- **FaceRecognition.action** - Action messages for the face_recognition_server
		
		#goal
        uint8   order_id 
		string  order_argument
		
		#result
		uint8   order_id
		string[]  names
		float32[] confidence
		
		#feedback
		uint8   order_id
		string[]  names
		float32[] confidence
		
- **FaceRecognitionClientGoal.msg**
	The `face_recognition_client` listens to messages of these format on `/face_rec_order` in order to send goals to the `face_recognition_server`.

		uint8   order_id 
		string  order_argument
	
	**Goals**:
	- *recognize once*
		order_id = 0 
	- *recognize continuously*
		order_id = 1 
	- *add training images*
		order_id = 2, order_argument = personName
	- *(re)train models*
		order_id = 3
	- *exit*
		order_id = 4
	
### Parametres
- *string* **algorithm** (default: "lbph")
	Face recognition algorithm to be used.
	Supported:
	- eigenfaces
	- fisherfaces
	- lbph
- *int* **no_training_images** (default: 20)
	Number of training images to be added at a time.
- *int* **max_faces** (default: 3)
	Max number of faces to be detected in the scene.
- *int* **display_window** (default: true)
	Whether to display camera feed window while training/recognition.

## Usage
- Make sure `roscore` is running
- Publish video stream to `/usb_cam/image_raw` 
	**Example** with the `usb_cam` package:
	```rosrun usb_cam usb_cam_node	```
- In separate terminals run the face_recognition server and client
	```rosrun face_recognition_server```
	```rosrun face_recognition_client```
- Publish goals to `/face_rec_order`
	**Example**:
	- Continuous recognition:
	```rostopic pub -1 /face_rec_order corobot_face_recognition/FaceRecognitionClientGoal -- 0 "unused"```
	- Adding training images:
	```rostopic pub -2 /face_rec_order corobot_face_recognition/FaceRecognitionClientGoal -- 2 "yourName"```
		 