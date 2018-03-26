# Como testar seu projeto sem um robô físico


Instale pacotes relacionados com vídeo:

	sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-openni-camera  ros-kinetic-usb-cam  ros-kinetic-async-web-server-cpp

Baixe um software que permite à OpenCV abrir diretamente a câmera:

	git clone https://github.com/OTL/cv_camera

Em seguida compile:

	cd ~/catkin_ws/src

	catkin_make

## Para rodar

Defina os parâmetros

	rosparam set cv_camera/device_id 0
	rosparam set cv_camera/cv_cap_prop_frame_width 640
	rosparam set cv_camera/cv_cap_prop_frame_height  480


Nota: se você precisar trabalhar com marcadores de realidade aumentada precisará [calibrar a câmera](calibrar_camera.md).


Para executar:

	rosrun cv_camera cv_camera_node

## Mudanças no código

As mudanças necessárias no código se resumem a uma linha: Troque o nome do tópico `/raspicam_node/image/compressed` por `/cv_camera/image_raw/compressed`

Passará a ficar assim:

	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

Boa sorte!



