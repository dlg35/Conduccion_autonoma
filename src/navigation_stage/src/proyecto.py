# -*- coding: utf-8 -*-
# from __future__ import print_function

import rospy
import smach_ros
import math
import actionlib
import sys

from std_msgs.msg import Int32, String
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus

# Nombre de los topics
TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'
TOPIC_COLOR = '/color_detected'

# Definimos los ángulos de visión
ANG_IZQ = 60*math.pi/180.0
ANG_DER = -ANG_IZQ
ANG_CENT = 0


class Base(State):
    def __init__(self):
        State.__init__(self, outcomes=['red', 'green', 'orange', 'blue', 'pink', 'base'])
        # Declaramos algunas variables necesarias para la ejecución
        self.color_detected = False 
        self.color_detectado = 0
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.subColor = rospy.Subscriber(TOPIC_COLOR, Int32, self.color_detected_callback)
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
    
    def execute(self, userdata):
        # Nos quedamos en espera hasta que detecte un color
        while not self.color_detected:
           1
        # Dependiendo del color el estado cambia a un nodo u otro
        if self.color_detectado.data == 1:
            return "red"
        elif self.color_detectado.data == 2:
            return "green" 
        elif self.color_detectado.data == 3:
            return"orange"  
        elif self.color_detectado.data == 4:
            return "blue" 
        elif self.color_detectado.data == 5:
            return "pink"
        else:
            return "base"

    
    def laser_callback(self, msg):
        # Hacemos que el robot se mueva aleatoriamente
        # Definimos los rangos de la derecha, izquierda y central
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        pos_cen = int((ANG_CENT-msg.angle_min)/msg.angle_increment)

        cmd = Twist()

        # Si la distancia minima por cualquiera de los dos lados del robot 
        # hasta la pared es menor que 1, giramos el robot hacia el lado contrario,
        # si las distancias laterales son mayores que 1, continuamos de frente
        if(msg.ranges[pos_der] < 0.8):
            cmd.linear.x = 0.4
            cmd.angular.z = 0.5
        elif(msg.ranges[pos_izq] < 0.8):
            cmd.linear.x = 0.4
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.8
            cmd.angular.z = 0

        # Si la distancia del centro es menor que 1, hacemos un giro total
        # hacia la derecha
        if(msg.ranges[pos_cen] < 1.0):
            cmd.linear.x = 0
            cmd.angular.z = -1

        # Publicamos las velocidades angulares y lineares 
        self.pub.publish(cmd)

    def color_detected_callback(self, msg):
        # Callback para la detección de color
        self.color_detectado = msg
        self.color_detected = True

# Si detectamos el color rojo
class rojo(State):
    def __init__(self):
        State.__init__(self, outcomes=['base', 'fin'])
        # Declaramos una variable para iniciar el tiempo 
        self.start_time = None
        self.detectado = False
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.sub = rospy.Subscriber(TOPIC_COLOR, Int32, self.color_detected_callback)
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
       

    def execute(self, userdata):
        # Iniciamos el tiempo
        self.start_time = rospy.Time.now()
        cmd = Twist()
        #self.detectado = False
        self.detectado = False
        
        try: 
            # Durante 5 segundos el robot se queda parado
            while (rospy.Time.now() - self.start_time).to_sec() < 5:
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.pub.publish(cmd)
        except rospy.ROSInterruptException:
            pass
        

        while self.detectado == False:
            rospy.sleep(10)
            cmd.linear.x = 0.5
            cmd.angular.z = 0
            self.pub.publish(cmd)

        # Volvemos al estado base
        return 'base'

    def color_detected_callback(self, msg):
        # Callback para la detección de color
        self.color = msg

        if(self.color.data == 1):
            self.detectado = False
        else:
            self.detectado = True


# Si detectamos el color verde
class verde(State):
    def __init__(self):
        State.__init__(self, outcomes=['base', 'fin'])
        # Declaramos una variable para iniciar el tiempo 
        self.start_time = None
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)

    def execute(self, userdata):
        # Iniciamos el tiempo
        self.start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        try: 
            # Durante 5 segundos se ejecuta el bucle; el movimiento del robot se ejecutara
            # gracias a la funcion del laser_callback, la cual hace que siga moviendose a 
            # la misma velocidad
            while (rospy.Time.now() - self.start_time).to_sec() < 5:
                # Pausa el bucle a la frecuencia definida.
                rate.sleep()  
        except rospy.ROSInterruptException:
            pass

        # Volvemos al estado base
        return 'base'

    def laser_callback(self, msg):
        # Hacemos que el robot se mueva aleatoriamente
        # Definimos los rangos de la derecha, izquierda y central
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        pos_cen = int((ANG_CENT-msg.angle_min)/msg.angle_increment)

        cmd = Twist()

        # Si la distancia minima por cualquiera de los dos lados del robot 
        # hasta la pared es menor que 1, giramos el robot hacia el lado contrario,
        # si las distancias laterales son mayores que 1, continuamos de frente
        if(msg.ranges[pos_der] < 0.8):
            cmd.linear.x = 0.6
            cmd.angular.z = 0.7
        elif(msg.ranges[pos_izq] < 0.8):
            cmd.linear.x = 0.6
            cmd.angular.z = -0.7
        else:
            cmd.linear.x = 1.2
            cmd.angular.z = 0

        # Si la distancia del centro es menor que 1, hacemos un giro total
        # hacia la derecha
        if(msg.ranges[pos_cen] < 1.0):
            cmd.linear.x = 0
            cmd.angular.z = -1.2

        # Publicamos las velocidades angulares y lineares 
        self.pub.publish(cmd)

# Si detectamos el color naranja
class naranja(State):
    def __init__(self):
        State.__init__(self, outcomes=['base', 'fin'])
        # Declaramos una variable para iniciar el tiempo 
        self.start_time = None
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)

    def execute(self, userdata):
        # Iniciamos el tiempo
        self.start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        try: 
            # Durante 5 segundos se ejecuta el bucle; el movimiento del robot se ejecutara
            # gracias a la funcion del laser_callback, la cual hace que siga moviendose a 
            # una velocidad reducida
            while (rospy.Time.now() - self.start_time).to_sec() < 5:
                # Pausa el bucle a la frecuencia definida.
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

        # Volvemos al estado base
        return 'base'

    def laser_callback(self, msg):
        # Hacemos que el robot se mueva aleatoriamente
        # Definimos los rangos de la derecha, izquierda y central
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        pos_cen = int((ANG_CENT-msg.angle_min)/msg.angle_increment)

        cmd = Twist()

        # Si la distancia minima por cualquiera de los dos lados del robot 
        # hasta la pared es menor que 1, giramos el robot hacia el lado contrario,
        # si las distancias laterales son mayores que 1, continuamos de frente
        if(msg.ranges[pos_der] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
        elif(msg.ranges[pos_izq] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0

        # Si la distancia del centro es menor que 1, hacemos un giro total
        # hacia la derecha
        if(msg.ranges[pos_cen] < 1.0):
            cmd.linear.x = 0
            cmd.angular.z = -1

        # Publicamos las velocidades angulares y lineares 
        self.pub.publish(cmd)

class giro_derecha(State):
    def __init__(self):
        State.__init__(self, outcomes=['base', 'fin'])
        # Declaramos una variable para iniciar el tiempo 
        self.start_time = None
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.sub = rospy.Subscriber(TOPIC_COLOR, Int32, self.color_detected_callback)
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)

    def execute(self, userdata):

        # Iniciamos el tiempo
        self.start_time = rospy.Time.now()
        self.detectado = False
        rate = rospy.Rate(10)
        cmd = Twist() 

        while self.detectado == False:
            1
        # Creamos una variable sobre la que iteraremos, para que el robot de un cuarto de vuelta
        # con una velocidad angulor de 1 tiene que hacerse el número de iteraciones especificadas
        i = 0
        while (i < 40000):
            i = i+1
            cmd.linear.x = 0
            cmd.angular.z = -1
            self.pub.publish(cmd)

        # Volvemos al estado base
        return 'base'

    def color_detected_callback(self, msg):
        # Callback para la detección de color
        self.color = msg
        
        if(self.color.data == 4):
            self.detectado = False
        else:
            self.detectado = True

    def laser_callback(self, msg):
        # Hacemos que el robot se mueva aleatoriamente
        # Definimos los rangos de la derecha, izquierda y central
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        pos_cen = int((ANG_CENT-msg.angle_min)/msg.angle_increment)

        cmd = Twist()

        # Si la distancia minima por cualquiera de los dos lados del robot 
        # hasta la pared es menor que 1, giramos el robot hacia el lado contrario,
        # si las distancias laterales son mayores que 1, continuamos de frente
        if(msg.ranges[pos_der] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
        elif(msg.ranges[pos_izq] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0

        # Si la distancia del centro es menor que 1, hacemos un giro total
        # hacia la derecha
        if(msg.ranges[pos_cen] < 1.0):
            cmd.linear.x = 0
            cmd.angular.z = -1

        # Publicamos las velocidades angulares y lineares 
        self.pub.publish(cmd)



class giro_izquierda(State):
    def __init__(self):
        State.__init__(self, outcomes=['base', 'fin'])
        # Declaramos una variable para iniciar el tiempo 
        self.start_time = None
        # Nos  suscribimos a los topics que vamos a usar en este estado
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.sub = rospy.Subscriber(TOPIC_COLOR, Int32, self.color_detected_callback)
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)

    def execute(self, userdata):

        # Iniciamos el tiempo
        self.start_time = rospy.Time.now()
        self.detectado = False
        rate = rospy.Rate(10)
        cmd = Twist() 

        while self.detectado == False:
            1
        # Creamos una variable sobre la que iteraremos, para que el robot de un cuarto de vuelta
        # con una velocidad angulor de 1 tiene que hacerse el número de iteraciones especificadas
        i = 0
        while (i < 40000):
            i = i+1
            cmd.linear.x = 0
            cmd.angular.z = 1
            self.pub.publish(cmd)

        # Volvemos al estado base
        return 'base'


    def color_detected_callback(self, msg):
        # Callback para la detección de color
        self.color = msg

        if(self.color.data == 5):
            self.detectado = False
        else:
            self.detectado = True

    def laser_callback(self, msg):
        # Hacemos que el robot se mueva aleatoriamente
        # Definimos los rangos de la derecha, izquierda y central
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        pos_cen = int((ANG_CENT-msg.angle_min)/msg.angle_increment)

        cmd = Twist()

        # Si la distancia minima por cualquiera de los dos lados del robot 
        # hasta la pared es menor que 1, giramos el robot hacia el lado contrario,
        # si las distancias laterales son mayores que 1, continuamos de frente
        if(msg.ranges[pos_der] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
        elif(msg.ranges[pos_izq] < 0.8):
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0

        # Si la distancia del centro es menor que 1, hacemos un giro total
        # hacia la derecha
        if(msg.ranges[pos_cen] < 1.0):
            cmd.linear.x = 0
            cmd.angular.z = -1

        # Publicamos las velocidades angulares y lineares 
        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("proyecto")
    sm = StateMachine(outcomes=['end'])

    with sm:
        # Creamos los estados del programa
        StateMachine.add('Base', Base(),transitions={'red':'rojo', 'green':'verde', 'orange':'naranja', 'blue': 'giro_derecha', 'pink':'giro_izquierda', 'base':'Base'})
        StateMachine.add('rojo', rojo(),transitions={'base':'Base', 'fin':'end'})
        StateMachine.add('verde', verde(), transitions={'base':'Base', 'fin':'end'})
        StateMachine.add('naranja', naranja(), transitions={'base':'Base', 'fin':'end'})
        StateMachine.add('giro_derecha', giro_derecha(), transitions={'base':'Base', 'fin':'end'})
        StateMachine.add('giro_izquierda', giro_izquierda(), transitions={'base':'Base', 'fin':'end'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
