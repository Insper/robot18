# Como instalar OpenCV no Ubuntu 16.04


Siga este guia para compilar a OpenCV direto a partir do código-fonte no Ubuntu para que funcione bem com o ROS
[http://wiki.ros.org/opencv3](http://wiki.ros.org/opencv3)


# Instruções



	git clone https://github.com/ros-gbp/opencv3-release


	cd opencv3-release


	git-bloom-generate -y rosdebian 



git-bloom-generate -y rosdebian --prefix kinetic  -i 3.3.1 --os-name ubuntu