#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import cormodule

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0



tolerancia_x = 30
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 300 # área da distancia ideal do contorno - note que varia com a resolução da câmera

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados




def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	


if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	#recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	
	# Para usar a webcam 
	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
				else:
					if dif_x > 0: # Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					else: # Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


## Classes - estados
# define state Foo
class Longe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ainda_longe','perto'])

    def execute(self, userdata):
        global velocidade_saida
        rospy.loginfo('Executing state LONGE')
        if media[1] > centro[1] - tolerancia_y:
        	# Avancar
            return 'ainda_longe'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
            return 'perto'


# define state Bar
class Andando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ainda_longe'])

    def execute(self, userdata):
        global velocidade_saida
        rospy.loginfo('Executing state ANDANDO')
        #comando para andar
        vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
        velocidade_saida.publish(vel)
        #rospy.sleep(0.05)

        return 'ainda_longe'

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'alinhando'])

    def execute(self, userdata):
        print ("angulo: ", ang)
        global velocidade_saida

        if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
            velocidade_saida.publish(vel)
        	return 'girando'
        if math.fabs(media[0]) < math.fabs(centro[0] + tolerancia_x):
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
            velocidade_saida.publish(vel)
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
        	return 'alinhou'


# main
def main():
    global velocidade_saida
    global buffer
    rospy.init_node('cor estados')

	# Para usar a webcam 
	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LONGE', Longe(), 
                               transitions={'ainda_longe':'ANDANDO', 
                                            'perto':'GIRANDO'})
        smach.StateMachine.add('ANDANDO', Andando(), 
                               transitions={'ainda_longe':'LONGE'})
        smach.StateMachine.add('GIRANDO', Girando(),
                                transitions={'alinhando': 'GIRANDO',
                                'alinhou':'LONGE2'})
        smach.StateMachine.add('LONGE2', Longe(), 
                               transitions={'ainda_longe':'ANDANDO2', 
                                            'perto':'terminei'})
        smach.StateMachine.add('ANDANDO2', Andando(), 
                               transitions={'ainda_longe':'LONGE2'})


    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()
