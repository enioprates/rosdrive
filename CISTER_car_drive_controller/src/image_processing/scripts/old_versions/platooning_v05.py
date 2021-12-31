#!/usr/bin/env python
#-----------------------------------------------------------------------
#platooning.py TV (target Vehicle) SV
#Codigo responsavel pelo controle do veiculo seguidor (SV)
#v0
#   Le os dados do TV (local) (param0) - TV (target Vehicle)
#   Le os dados atuais do SV (param1) - SV (subject vehicle)
#   Atua sobre o SV
#v1
#   Atualizando termos e variaveis para nomenclatura padrao
#v2
#   Introduzindo controle de distancia/tempo
#v3
#   Introduzindo controle angulacao
#v4
#   Ajustando delay entre acoes de control 0.05
#v5
#   Adicionando lista de posicoes
#   Adicionando controle do Heading
#-----------------------------------------------------------------------
import rospy
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
#from image_processing.msg import AckermannDriveStamped
#from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math
import numpy as np
import time as tp
import csv
import numpy as np

#------------------------------------------------------------------------

#Publication Topic

#------------------------------------------------------------------------

#global variables  
#dados do TV a ser seguido --------------------------------------------
longitude_TV = 0.0                                       #longitude
latitude_TV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_TV = 0.0											#velocidade do veiculo (m/s)
steering_TV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_TV = 0.0                                        #percentual do acelerador (0 - 1)
brake_TV = 0.0                                           #percentual do freio (0 - 1)

#dados do proprio veiculo --------------------------------------------
longitude_SV = 0.0                                       #longitude
latitude_SV = 0.0                                        #latitude 
heading_SV = 0.0                                         #heading (angulacao global do veiculo)
speed_SV = 0.0											#velocidade do veiculo
steering_SV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual do freio (0 - 1)

longitude_old = 0.0
latitude_old = 0.0
heading_SV = 0.0                                         #heading old (angulacao global do veiculo)
speed_SV = 0.0											#velocidade old do veiculo
steering_SV = 0.0                                        #steering old (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual old do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual old do freio (0 - 1)

#variaveis de controle --------------------------------------------
kp_vel = 1
ki_vel = 0
kd_vel = 0

vel_integral = 0.0
vel_deriv = 0.0
erro_vel_old = 0.0
erro_vel = 0.0

kp_theta = 9.0
ki_theta = 0.1
kd_theta = 3.5

theta_integral = 0.0
theta_deriv = 0.0
theta_erro_old = 0.0
theta_error = 0.0


#variaveis gerais -------------------------------------------------
TV_topic = 0                                           #topico do TV
SV_topic = 0                                         #topico do SV
publication_topic = 0                                      #topico a publicar dados do veiculo

time_distance_control=0.0
time_direction_control = 0.0
count = 0
lap = 0
count_lap = 0
error_steer = 0.0
control_error_steer  = 0.0
steer_integral  = 0.0
steer_deriv  = 0.0
status  = 0

TV_position_vector = np.empty([1,3])

#defines------------------------------------------------------------
MAX_SPEED = 30.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 30.0				#maxima angulacao da roda em graus
MIN_STEER = -30.0				#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6					#converte de m/s para Km/h
EDGE_LOW_SPEED	= 0.05		#ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
EDGE_ANGLE = 0.001
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#

MINIMUM_DISTANCE = 5        #distancia de seguranca - m
SAFETY_TIME = 2             #tempo de seguranca - s
MAX_ERRO_INTEGRADOR_VEL = 50
MAX_ERRO_INTEGRADOR_THETA = 50
ZERO = 0

MINIMUM_DT = 0.05
#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(data)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(acao_vel, v_TV):
	
	erro_vel_norm = (acao_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)				                #acao de controle
	accel_control = max(accel_control,-1.0)				                #acao de controle

	if(abs(accel_control) < EDGE_LOW_SPEED and v_TV < EDGE_LOW_SPEED):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: distance_control(v_TV, v_SV, distance_error)
#   Control the distance between the TV and the SV using the speed of the SV	
#   v_TV -> TV speed
#   v_SV -> SV Speed
#   d_error -> Error between the desired distance and the real distance
#   Return: control_vel (Data responsible for the new desired speed in m/s)
#-----------------------------------------------------------------------
def distance_control (v_TV, v_SV, d_error):
    #Global variables-----------------
    global vel_integral
    global vel_deriv
    global erro_vel_old
    global time_distance_control
    global erro_vel
    
    #Calcule dt------------------------------------------------------------
    #used to determine the time betwwen interactions from the program
    time_now = tp.time()									#armazena o tempo atual
    dt = time_now - time_distance_control								    #tempo entre iteracoes
    if (dt >= MINIMUM_DT):
        time_distance_control = time_now										#armazena o tempo atual para a proxima iteracao

        #Calculate the erro_vel taking into consideration that the speed is given in m/s
        #The Erro_vel is given by the difference between the speeds of TV and SV + the adjust 
        # factor given by the distance_error
        erro_vel = (v_TV - v_SV) + d_error                      #erro de velocidade 
        
        #print "vel_TV: ", v_TV
        #print "vel_SV: ", v_SV
        if (erro_vel >= EDGE_LOW_SPEED):                #se erro de velocidade significativo
            #print "VEL_DIFERENTE"
            #determina valor do integrador
            if (abs(vel_integral) >= MAX_ERRO_INTEGRADOR_VEL or dt>10):
                vel_integral = ZERO
            else:
                vel_integral = vel_integral + erro_vel * dt
            
            #valor do derivativo
            vel_deriv = erro_vel_old - erro_vel
            erro_vel_old = erro_vel
            control_vel = ( kp_vel*erro_vel + ki_vel*vel_integral +kd_vel*vel_deriv)
            #print "Control_vel: ", control_vel
        else:
            #print "VEL_IGUAL"
            erro_vel = 0.0
            control_vel = 0.0
        return control_vel
    else:
        return 0.0

#-----------------------------------------------------------------------
#FUNCAO: direction_control(heading_TV, heading_SV)
#   Control the direction of the SV using the heading of the TV	
#   h_TV -> TV speed
#   h_SV -> SV Speed
#   Return: theta_control 
#-----------------------------------------------------------------------
def direction_control (h_TV, h_SV):
    #Global variables-----------------
    global theta_integral
    global theta_deriv
    global theta_erro_old
    global time_direction_control
    global theta_error
   
    #Calcule dt------------------------------------------------------------
    #used to determine the time between interactions from the program
    time_now = tp.time()									#armazena o tempo atual
    dt = time_now - time_direction_control								    #tempo entre iteracoes
    if (dt >= MINIMUM_DT):
        time_direction_control = time_now										#armazena o tempo atual para a proxima iteracao

        print "h_TV: ", h_TV
        print "h_SV: ", h_SV
        
        if (h_TV < 0.0):
            theta_TV = 2*math.pi + h_TV
        else:
            theta_TV = h_TV
        if (h_SV < 0.0):
            theta_SV = 2*math.pi + h_SV
        else:
            theta_SV = h_SV
        print "Theta_TV: ", theta_TV
        print "Theta_SV: ", theta_SV
        
        theta_error = theta_TV - theta_SV
        if (theta_error > math.pi):
            theta_error = (abs(h_TV) + abs(h_SV))* (h_TV/abs(h_TV))
        
        print "Theta_error: ", theta_error

        if (abs(theta_error) >= EDGE_ANGLE):                #se erro de velocidade significativo
            print "ANGLE_DIFERENTE"
            #determina valor do integrador
            if (abs(theta_integral) >= MAX_ERRO_INTEGRADOR_THETA or dt>10):
                theta_integral = ZERO
            else:
                theta_integral = theta_integral + theta_error * dt
            
            #valor do derivativo
            theta_deriv = theta_erro_old - theta_error
            theta_erro_old = theta_error

            #control action
            theta_control = ( kp_theta*theta_error + ki_theta*theta_integral +kd_theta*theta_deriv)
            
            if (theta_control > MAX_STEER*ANG_TO_RAD):
                theta_control = MAX_STEER*ANG_TO_RAD
            elif (theta_control < - MAX_STEER*ANG_TO_RAD):
                theta_control = -MAX_STEER*ANG_TO_RAD
            print "theta_control: ", theta_control
        else:
            print "ANGLE_IGUAL"
            theta_error = 0.0
            theta_control = 0.0
        return theta_control
    else:
        return 0.0
#-----------------------------------------------------------------------
#FUNCAO: compare_position():
#	
#-----------------------------------------------------------------------
def compare_position():
    global TV_position_vector
    global longitude_SV
    global latitude_SV
    global heading_SV
    dist = 0
    dist_old = 999
    pos = 0

    for i in range(TV_position_vector.shape[0]):
        print "i: ", i
        print "TV_position_vector: ", TV_position_vector[i,1], TV_position_vector[i,0]
        print "SV:                 ", longitude_SV, latitude_SV
        dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
        print "Dist: ", dist
        print "Dist_old: ", dist_old
        if dist_old < dist:
            pos = i
            break
        else:
            dist_old = dist
    
    if (dist_old <= 5):
        heading_out = TV_position_vector[pos,2]
        TV_position_vector = TV_position_vector[pos:,:]
    else:
        heading_out = heading_SV

    return heading_out


#-----------------------------------------------------------------------
#FUNCAO: vehicle_general_control()
#	
#-----------------------------------------------------------------------
def vehicle_general_control():
    #TV data
    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    #SV data
    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1)

    global publication_topic
    global erro_vel
    global TV_position_vector

    pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
    msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
    msg.header.stamp = rospy.Time.now();					#tempo SVizado da leitura
    msg.header.frame_id = "base_link";						
    msg.steer = 0 * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS

    #calcula distancia
    distance = ((longitude_TV - longitude_SV)**2 + (latitude_TV - latitude_SV)**2)**0.5
    #print "Distance", distance
    maximum_distance = MINIMUM_DISTANCE + speed_TV * SAFETY_TIME
    distance_error = distance - maximum_distance
    
    print "------------------------------------------------------"
    TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
    print "TV_position_vector shape: ", TV_position_vector.shape
    #print TV_position_vector
    
    #ajusta velocidade e aceleracao
    if (speed_TV >= EDGE_LOW_SPEED and distance_error >= 0.0):
        print "TV moving"
        acao_velocidade = distance_control (speed_TV, speed_SV, distance_error)
        
        msg.throttle, msg.brake =calc_throttle_brake(acao_velocidade, speed_TV)
    else:
        print "TV stopped"
        msg.throttle = 0.0
        msg.brake = 0.0
    
    #ajusta o Heading
    if (speed_TV >= EDGE_LOW_SPEED and speed_SV>= EDGE_LOW_SPEED):
        heading_compare = compare_position()
        if (heading_compare <> heading_SV):
            msg.steer = direction_control (heading_compare, heading_SV)

    #print "throttle", msg.throttle
    #print "brake", msg.brake
    #Atualiza topico carro
    pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'


def SV_info(data):

    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1

    #print 'SV_INFO'

    latitude_SV = data.latitude
    longitude_SV = data.longitude
    heading_SV = data.heading_headingValue
    speed_SV = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_SV = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_SV = data.gasPedalPercent_Value
    brake_SV = data.brakePedalPercent_Value
    vehicle_general_control()

def TV_info(data):

    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    global count
    global lap
    global count_lap

    global TV_position_vector
    #print 'TV_INFO'

    latitude_TV = data.latitude
    longitude_TV = data.longitude
    heading_TV = data.heading_headingValue
    speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_TV = data.gasPedalPercent_Value
    brake_TV = data.brakePedalPercent_Value

    


def listener():
    global TV_topic
    global SV_topic
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print TV_topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TV_topic,CAM_simplified,TV_info)
    rospy.Subscriber(SV_topic,CAM_simplified,SV_info)

    #print "Aqui"

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #global TV_topic
    #global SV_topic
    #global publication_topic
    param = sys.argv[1:]                                #le os dados passados como argumentos (TV SV)
                                                        #param[0] - TV
                                                        #param[1] - SV
    #TODO - tratamento de erros para parametros de entrada
    #TODO - Tornar generico parametros de leitura
    TV_topic = "/" + param[0] +"/carINFO"                   #cria a string do topico do TV a ser lido
    SV_topic = "/" + param[1] +"/carINFO"                   #cria a string do topico do SV a ser lido
    publication_topic = "/" + param[1] +"/prius"            #cria a string do topico do SV a ser publicado (controle)
    listener()
