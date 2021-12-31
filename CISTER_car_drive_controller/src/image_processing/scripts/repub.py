#!/usr/bin/env python
# /******************************************************************************************************
#  * Copyright 2020 Enio Vasconcelos Filho (eniopvf@gmail.com, enio.filho@ifg.edu.br, enpvf@isep.ipp.pt)
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License");
#  * you may not use this file except in compliance with the License.
#  * You may obtain a copy of the License at
#  *
#  *     http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS,
#  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  * See the License for the specific language governing permissions and
#  * limitations under the License.
#  *
# *******************************************************************************************************/
#-----------------------------------------------------------------------
#Listener.py
#Codigo responsavel por guardar os dados de posicao do veiculo
#v00
#	Codigo inicial, gravando apenas dados de posicao, velocidade e angulacao do veiculo
#v01	
#   Gravando dados dos sonares
#v02
#	Retirando gravacao dos sonares
#	Gravando posicoes de carN (entrada de dados)
#v03
#v04
#	Change the event that triggers the record
#v05
#	adding new cars
#	adding sonar reads for CAR1
#	changing the CAR parameters
#-----------------------------------------------------------------------

import rospy
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
from image_processing.msg import AckermannDriveStamped
from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
from ros_its_msgs.msg import OMNET_CAM
from ros_its_msgs.msg import Sonar						#msg do tipo Sonar
import math
import numpy as np
import time as tp
import csv

#global variables  
TV_name = 0
longitude_TV = 0.0                                       #longitude
latitude_TV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_TV = 0.0											 #vehicle speed (m/s)

real_followers = 4

TIME_SPACE = 0.01

ON = 1
OFF = 0


def car_info_TV_simplified(data):
	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV

	global param

	#print 'CAR_INFO'
	speed_TV = data.speed						# salva o valor da velocidade no instante
	latitude_TV = data.latitude
	longitude_TV = data.longitude
	heading_TV = data.heading

	#Vehicle controller 
	#print real_followers
	#print int(param[1])
	for i in range(real_followers+1,int(param[1])+1):
		
		pub = rospy.Publisher('car'+ str(i)  +'/omnetCAM', OMNET_CAM, queue_size=5)				#topico de controle do carro
		msg = OMNET_CAM()																	#Define msg como um dado do tipo Control -> comando de controle do veiculo
		msg.car_name = "/car"+str(i)+"/"
		#print str(i) + ' ' + msg.car_name
		msg.latitude = latitude_TV
		msg.longitude = longitude_TV
		#msg.speed = speed_TV
		#msg.heading = heading_TV
		pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'


def listener():
	global param
	global SV_name
	global TV_name

	TV_name = "/" + param[0]

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)
	print 'LISTENER! - ' + param[0] + ' - ' + param[1]
	rospy.Subscriber(TV_name + '/omnetCAM',OMNET_CAM,car_info_TV_simplified)

    	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	
	param = sys.argv[1:]
	time_base = tp.time()
	listener()
