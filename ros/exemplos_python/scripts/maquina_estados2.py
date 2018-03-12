#!/usr/bin/env python

import roslib
import rospy
import sys
import smach
import smach_ros
import rospy
import numpy
from numpy import linalg
import transformations
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header

x = 0
y = 0
z = 100
id = 0
ang = -500
x_desejado = 0.12
y_desejado = 0.10
z_desejado = 1.00
tfl = 0

buffer = tf2_ros.Buffer()

def recebe(msg):
    global x
    global y
    global z
    global ang
    for marker in msg.markers:
        x = round(marker.pose.pose.position.x, 2)
        y = round(marker.pose.pose.position.y, 2)
        z = round(marker.pose.pose.position.z, 2)
        if marker.id == 100:
            header = Header(frame_id= "ar_marker_100")
            print("id:", marker.id)
            can_transf = buffer.can_transform("base_link", "ar_marker_100", rospy.Time(0))
            print("can:",can_transf)
            # if can_transf == False:
            #     sys.exit(0)
            # else:
            #     print("Can Transf = True")
            trans = buffer.lookup_transform("base_link", "ar_marker_100", rospy.Time(0))
            t = transformations.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            m = numpy.dot(r,t)
            v2 = numpy.dot(m,[0,0,1,0])
            v2_n = v2[0:-1]
            n2 = v2_n/linalg.norm(v2_n)
            cosa = numpy.dot(n2,[1,0,0])
            ang = math.degrees(math.acos(cosa))
            print ("angulo", ang)

# define state Foo
class Longe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ainda_longe','perto'])

    def execute(self, userdata):
        global velocidade_saida
        rospy.loginfo('Executing state LONGE')
        if z > z_desejado:
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
        if ang < 110:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
            velocidade_saida.publish(vel)
            return ('alinhando')
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
            return ('alinhou')    

# main
def main():
    global velocidade_saida
    global buffer
    rospy.init_node('smach_example_state_machine')
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    tfl = tf2_ros.TransformListener(buffer)

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
