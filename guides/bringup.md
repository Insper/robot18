
## No PC
Conectar-se à raspberry:

Você precisa do IPBerry e da senha (pergunte a senha da semana ao professor)

	ssh pi@IPBerry


## Na Raspberry


Certifique-se de que o cabo da bateria (plug J2 vermelho) está conectado.

Ligue o equipamento na chave de liga-desliga


Inicie na Raspberry:

Primeiro o gerenciador de sessões:

	screen

Para todas as instruções abaixo a separação por vírgula quer dizer que o primeiro atalho deve ser realizado, depois solta-se o teclado antes de proceder ao segundo atalho.

Agora crie 3 sub-sessões dentro da sessão:

`Ctrl A, C`
`Ctr A, C`
`Ctrl A, C`

Agora volte para a segunda sessão:

`Ctrl A, "`

Note que em algumas plataformas para que a tecla de `"` seja considerada apertada é necessário apertar barra de espaço em seguida.

Deve aparecer uma lista de sessões


Em cada uma das sessões dê um dos seguintes comandos:

	roslaunch turtlebot3_bringup turtlebot3_core.launch

	roslaunch turtlebot3_bringup turtlebot3_lidar.launch

	roslaunch raspicam_node camerav2_640x480_30fps.launch

Ao final, dê

`Ctrl A, D`

Este comando faz o detach da sessão e permite que você encerre o SSH sem matar os programas

Digite na linha de comando:

	exit

## De volta no PC

Crie a variável de ambiente do PC

	export ROS_IP=`hostname -I`


Certifique-se de que todo o software do Turtlebot e do curso está atualizado:

	cd ~
	cd catkin_ws/src
	git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin
	cd turtlebot3
	git pull
	cd ../turtlebot3_msgs
	git pull
	cd ../turtlebot3_simulations
	git pull
	cd ../robot18
	git pull
	cd ~/catkin_ws
	catkin_make


Crie a variável de ambiente para conectar-se à Raspberry Pi:

	export ROS_MASTER_URI=http://IPBerry:11311

Agora execute:

	roscore

Depois:

	roslaunch turtlebot3_bringup turtlebot3_remote.launch

Depois, finalmente:

	rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz


Sempre que a câmera estiver invertida, rode para ajustá-la:

	rosrun rqt_reconfigure rqt_reconfigure


Se quiser teleoperar o robô faça:

	rosrun rqt_image_view rqt_image_view


Para acionar o comando por teclas:
	roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


## No robô novamente, para desligar

Conecte-se ao ssh

Digite:

	shutdown -P now

Assim que as luzes da Raspberry passarem a piscar desligue a energia no cabo


Fonte: [Manual do Bringup do Turtlebot3 - documentação oficial](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup)


	







