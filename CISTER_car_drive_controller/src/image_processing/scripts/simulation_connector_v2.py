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
#Simulation_connector.py
#Codigo responsavel pela ligacao entre o sistema de controle e o simulador do veiculo
#v0
#	Codigo alterado para ler os dados de velocidade do topico carINFO
#	Criada a funcao car_info
#v1
#	Incorporado o codigo do distance.py
#v2
#	Convertendo velocidade em aceleracao e frenagem
#v3
#	Tratando o erro de angulacao
#v4
#	Alterando equacao de controle
#v5
#	(OLD) Salvando dados no csv (Influi no desempenho! Melhor deixar em NO separado)
#	Enviando dados para topico error_control
#v6
#	Limitando o crescimento do fator de integracao
#	Diminuindo o tempo de resposta pela metade
#	PID funcionando! (Curva e Reta)
#v7
#	Comentando o texto
#	Substituindo magical numbers
#v7.1
#	lendo sonares
#	criando funcao separada do PID
#v7.3 
#	desvio de multiplos obstaculos (carros)
#v7.4
#	acrescentando sonares laterais
#v7.5
#	mudando o topico de publicacao do carro de drive_param para car1/drive_param
#	mudando limitacao de velocidade maxima
#v7.6
#	deixando de publicar no topico de ERROS
#v8.0
#	reduzindo velocidade em curvas   (REDUCED_SPEED)
#v8.2
#	inicializando velocidade
#	adding the CAR parameter
#v8.3
#	adding a time-based stop condition
#	PID adjust (40 and 60 Km/h)
#	More laps
#-----------------------------------------------------------------------

#Import commands
import rospy												#biblioteca de python do ROS
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords						#leitura de coordenadas do carro
from image_processing.msg import AckermannDriveStamped		#RETIRAR
from image_processing.msg import error_control				#parametros para tratamentos de erro (e impressao em arquivo)
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
from ros_its_msgs.msg import Sonar						#msg do tipo Sonar
import math								
import numpy as np
import time as tp
import csv
#------------------------------------------------------------------------
#Publication Topic
#pub = rospy.Publisher('car1/prius', Control, queue_size=10)				#topico de controle do carro
#pub_2 = rospy.Publisher('car1/drive_parameters', drive_param, queue_size=1)	#parametros de velocidade do veiculo
#pub_3 = rospy.Publisher('car1/error_control', error_control, queue_size=1) 	#parametros de erro para controle e impressao
#------------------------------------------------------------------------

#global variables  
speed_TV = 0.0											#velocidade global do veiculo
status = 0													#indica se o veiculo esta em velocidade normal ou reduzindo
range_LF = 0.0												#variaveis de leitura do sonar
range_LM = 0.0												#variaveis de leitura do sonar
range_RM = 0.0												#variaveis de leitura do sonar
range_RF = 0.0												#variaveis de leitura do sonar
range_side_L = 0.0
range_side_R = 0.0
max_range_LF = 0.0
max_range_LM = 0.0
max_range_RM = 0.0
max_range_RF = 0.0
max_range_side_L = 0.0
max_range_side_R = 0.0


#PID de velocidade
kp_vel=	10.8		#10.8				#Kp
ki_vel=	2.16			#2.16				#Ki
kd_vel=	0.135			#0.135				#Kd

vel_integral = 0.0
vel_deriv = 0.0

#PID das rodas
kp_steer_l= 10.0						#Kp 40 km - 3.2			kp 50 km 5.5  5				18.0
ki_steer_l= 0.0							#Ki 40 km - 0.01		Ki 50 km 0.01   0			00.0
kd_steer_l= 0.0							#Kd 40 km - 0.2			Kd 50 km 0.4  0.45			07.0

steer_integral = 0.0
steer_deriv = 0.0			

#used variables
tp_corr = 1 							#segundos  - tempo correcao
time1 = 0.0								#variavel de contagem do tempo etre iteracoes (dt)
time_speed = 0.0
elapsed_time_TV = 0.0
old_time_TV = 0.0
longitude_TV = 0.0
latitude_TV = 0.0

#parametros Iniciais
velocity = 0.0							#velocidade inicial do veiculo
prev_error_steer=0.0					#erro anterior de angulacao
prev_error_vel=0.0						#erro anterior de velocidade
prev_theta = 0.0						#angulo anterior (usar no derivativo)
error_steer=0.0							#erro atual de angulacao
error_vel=0.0							#erro atual de velocidade
erro_vel = 0.0
theta_error = 0.0
angle=0.0								#angulo do veiculo
count = 0								#variavel de teste de numero de iteracoes
stop_condition = 0						#parametro de parada do veiculo
count_TV = 0
TV_name = 0

ON = 1
OFF = 0

REDUCED_SPEED = OFF
SONAR = OFF

MAX_SPEED = 100.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 30.0				#maxima angulacao da roda em graus
MIN_STEER = -30.0				#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6					#converte de m/s para Km/h

TIME_SPACE = 0.01
EDGE_LOW_SPEED	= 0.05		#ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#
MIN_SLOPE = 1.0				#indica deteccao de CURVA (<=1.0) OU RETA 
MAX_ERRO_INTEGRADOR	= 30.0 #valor maximo do I para correcao - steering
MAX_ERRO_INT_SPEED = 50 

GRAVA_SONAR = 1
NORMAL = 1					#status da conducao 
CORRECAO = 0				#status da conducao
ZERO = 0					#
FRENTE = 1					#direcoes de obstaculos
ESQUERDA = 2				#direcoes de obstaculos
DIREITA = 3					#direcoes de obstaculos
LADO_DIREITO = 4			#direcoes de obstaculos
LADO_ESQUERDO = 5			#direcoes de obstaculos
INDEFINIDO = 6				#direcoes de obstaculos
LIVRE = 0					#direcoes de obstaculos
DESVIO = 15.0				#angulo de desvio em caso de obstaculo

COLLISION = 0.40

STOP_VEHICLES = ON			#variable to stop the vehicles
STOP_LATITUDE = 410.0		#OVAL: 290.0	#CURVE: 410.0		#obstAvoid: 242.54		#CURVES: 150.05
STOP_LONGITUDE = 45.85		#OVAL: 3.75		#CURVE: 45.85		#obstAvoid: 163.08		#CURVES: 314.86
STOP_LAP = 0				#time to stop the vehicles
time_base = 0										#start time of the system

#LAP_LAT_V = [[383.05, 45.56],[234.60,162.89],[150.05, 216.63],[150.05,314.86],[52.47,290.75],[52.47,100.73],[110.30,45.56]]
LAP_LAT_V = [[160.0, 45.56],[250.0, 45.56],[360.0, 45.56],[400.00, 45.56],[234.60,162.89],[150.05, 216.63],[150.05,314.86],[52.47,290.75],[52.47,100.73],[110.30,45.56]]


INITIAL_SPEED = ON			#define initial speed
#---------------just one speed
TARGET_SPEED = 14.00 		#target speed (approx 50KM/h)
VEL_CRUZEIRO = TARGET_SPEED * MS_TO_KMH			#KM/H	50
VEL_FRENAGEM = TARGET_SPEED * MS_TO_KMH			#KM/H	30

SAME_SPEED = 12.00

#---------------more then one speed
TARGET_SPEED_1 = SAME_SPEED 		#target speed (approx 60KM/h)  	16		16
VEL_CRUZEIRO_1 = TARGET_SPEED_1 * MS_TO_KMH			#KM/H	50
VEL_FRENAGEM_1 = TARGET_SPEED_1 * MS_TO_KMH			#KM/H	30

TARGET_SPEED_2 = SAME_SPEED 		#target speed (approx 40KM/h)  14		14
VEL_CRUZEIRO_2 = TARGET_SPEED_2 * MS_TO_KMH			#KM/H	50
VEL_FRENAGEM_2 = TARGET_SPEED_2 * MS_TO_KMH			#KM/H	50

TARGET_SPEED_3 = SAME_SPEED 		#target speed (approx 40KM/h)  12		12
VEL_CRUZEIRO_3 = TARGET_SPEED_3 * MS_TO_KMH			#KM/H	43
VEL_FRENAGEM_3 = TARGET_SPEED_3 * MS_TO_KMH			#KM/H	43

TARGET_SPEED_4 = SAME_SPEED 		#target speed (approx 70KM/h)  16		16
VEL_CRUZEIRO_4 = TARGET_SPEED_4 * MS_TO_KMH			#KM/H	60
VEL_FRENAGEM_4 = TARGET_SPEED_4 * MS_TO_KMH			#KM/H	60

TARGET_SPEED_5 = SAME_SPEED 		#target speed (approx 70KM/h)  14		14
VEL_CRUZEIRO_5 = TARGET_SPEED_5 * MS_TO_KMH			#KM/H	60
VEL_FRENAGEM_5 = TARGET_SPEED_5 * MS_TO_KMH			#KM/H	60

TARGET_SPEED_6 = SAME_SPEED 		#target speed (approx 70KM/h)  17		16
VEL_CRUZEIRO_6 = TARGET_SPEED_6 * MS_TO_KMH			#KM/H	60
VEL_FRENAGEM_6 = TARGET_SPEED_6 * MS_TO_KMH			#KM/H	60

TARGET_SPEED_7 = SAME_SPEED 		#target speed (approx 70KM/h)  13		14
VEL_CRUZEIRO_7 = TARGET_SPEED_7 * MS_TO_KMH			#KM/H	60
VEL_FRENAGEM_7 = TARGET_SPEED_7 * MS_TO_KMH			#KM/H	60

TARGET_SPEED_8 = SAME_SPEED 		#target speed (approx 70KM/h)  22		25
VEL_CRUZEIRO_8 = TARGET_SPEED_8 * MS_TO_KMH			#KM/H	60
VEL_FRENAGEM_8 = TARGET_SPEED_8 * MS_TO_KMH			#KM/H	60

#------------------ ACCELERATION + BRAKE

# TARGET_SPEED_1 = 28.00 		#target speed (approx 60KM/h)  	16		16
# VEL_CRUZEIRO_1 = TARGET_SPEED_1 * MS_TO_KMH			#KM/H	50
# VEL_FRENAGEM_1 = TARGET_SPEED_1 * MS_TO_KMH			#KM/H	30

# TARGET_SPEED_2 = 28.00 		#target speed (approx 40KM/h)  14		14
# VEL_CRUZEIRO_2 = TARGET_SPEED_2 * MS_TO_KMH			#KM/H	50
# VEL_FRENAGEM_2 = TARGET_SPEED_2 * MS_TO_KMH			#KM/H	50

# TARGET_SPEED_3 = 28.00 		#target speed (approx 40KM/h)  12		12
# VEL_CRUZEIRO_3 = TARGET_SPEED_3 * MS_TO_KMH			#KM/H	43
# VEL_FRENAGEM_3 = TARGET_SPEED_3 * MS_TO_KMH			#KM/H	43

# STOP_LAP = 0				#time to stop the vehicles
#---------------------


count_lap_TV = 0
lap_TV = 0




#global
publication_topic = 0
car_name_TV = 0

#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(s_sp, s_TV) -> s_sp esta em KM/h e s_TV em m/s
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake_v2(s_sp, s_TV):
	global TARGET_SPEED
	global time_base
	global longitude_TV
	global latitude_TV
	global stop_condition
	global lap_TV
	global vel_integral
	global vel_deriv
	global prev_error_vel
	global count
	global time_speed
	global erro_vel

	debug = OFF

	#define o dt--------------------------------------------------------------------------------------------------------------------------------------
	time_now = tp.time()										#armazena o tempo atual
	if (count == 0):
		dt_speed = 0.0											#inicializa a variavel
	else:	
		dt_speed = time_now - time_speed						#tempo entre iteracoes
	time_speed = time_now										#armazena o tempo atual para a proxima iteracao
	erro_vel = s_sp/3.6  - s_TV									#calcula o erro de velocidade -> "/3.6" converte para m/s
	
	if debug:
		print 's_sp: ',s_sp,'s_sp/3.6: ',s_sp/3.6,' s_TV: ',s_TV

	#erro_vel = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)		#normaliza o erro (0 - 1)
	#if abs(erro_vel)>=1.0:
	#	erro_vel = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)		#normaliza o erro (0 - 1)

	if abs(erro_vel) <= 0.1:									#zerando erro do integrador
		vel_integral = ZERO
	elif abs(erro_vel) >= MAX_ERRO_INT_SPEED:					#limitando o integrador
		if (vel_integral) > 0.0:
			vel_integral = MAX_ERRO_INT_SPEED
		else:
			vel_integral = - MAX_ERRO_INT_SPEED
	else:	
		vel_integral = vel_integral + erro_vel * dt_speed		#increementando o integrador
	
	vel_deriv = (prev_error_vel - erro_vel)/dt_speed			#atualizando o derivativo

	control_error_vel = (kp_vel*erro_vel + ki_vel * vel_integral + kd_vel * vel_deriv)

	if debug:
		print 'erro_vel: ', erro_vel,' Control: ',control_error_vel, ' integ: ', vel_integral, ' deriv: ',vel_deriv

	accel_control = min(control_error_vel,1.0)					#acao de controle
	accel_control = max(accel_control,-1.0)						#acao de controle

	if debug == ON and lap_TV > 1:
		print "s_sp: ", s_sp
		print "s_sp/3.6: ", s_sp/3.6
		print "s_TV: ", s_TV

	
	if(abs(accel_control) < EDGE_LOW_SPEED and s_sp == MIN_VEL):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem

	if STOP_VEHICLES and lap_TV >= STOP_LAP:		#function to stop the vehicles to test the brakes
		#print ("STOP! ")
		if (abs(longitude_TV-STOP_LONGITUDE) < 2 and abs(latitude_TV-STOP_LATITUDE) < 2) or stop_condition:  	
			#print ("STOP   2! ")
			aceleracao = 0.0
			freio = 1.0
			stop_condition = ON
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: calc_PID(data)
#	Executa os comandos do PID
#	Retorna o erro da variavel de controle
#-----------------------------------------------------------------------
def calc_PID( theta_error, dt, slope, x2, x1):
	global steer_integral
	global prev_theta
	global steer_deriv
	#if abs(steer_integral) >= MAX_ERRO_INTEGRADOR or abs(theta_error) <= 0.1:			#zerando erro do integrador
	if abs(theta_error) <= 0.1:			#zerando erro do integrador
		steer_integral = ZERO
	elif abs(steer_integral) >= MAX_ERRO_INTEGRADOR:
		if (steer_integral) > 0.0:
			steer_integral = MAX_ERRO_INTEGRADOR
		else:
			steer_integral = - MAX_ERRO_INTEGRADOR
	else:	
		steer_integral = steer_integral + theta_error * dt								#increementando o integrador
	
	steer_deriv = (prev_theta - theta_error)/dt
	#steer_deriv	= shortest_angular_distance(prev_theta,theta_error)/dt
	control_error_steer = - (kp_steer_l*theta_error + ki_steer_l * steer_integral + kd_steer_l * steer_deriv)

	#print 'theta: ', theta_error,' Control: ',control_error_steer, ' integ: ', steer_integral, ' deriv: ',steer_deriv
	
	return control_error_steer

#-----------------------------------------------------------------------
#FUNCAO: sonar_XX(data)
#	Le os dados do sonar do veiculo
#-----------------------------------------------------------------------
def sonar_front_LF(data):
	global range_LF
	global max_range_LF
	range_LF = data.range
	max_range_LF = data.max_range
def sonar_front_LM(data):
	global range_LM
	global max_range_LM
	range_LM = data.range
	max_range_LM = data.max_range
def sonar_front_RM(data):
	global range_RM
	global max_range_RM
	range_RM = data.range
	max_range_RM = data.max_range
def sonar_front_RF(data):
	global range_RF
	global max_range_RF
	range_RF = data.range
	max_range_RF = data.max_range
def sonar_side_R(data):
	global range_side_R
	global max_range_side_R
	range_side_R = data.range
	max_range_side_R = data.max_range
def sonar_side_L(data):
	global range_side_L
	global max_range_side_L
	range_side_L = data.range
	max_range_side_L = data.max_range


#-----------------------------------------------------------------------
#FUNCAO: trata_sonar(data)
#	Le os dados do sonar do veiculo
# 	reyorna a posicao do obstaculo na pista
#-----------------------------------------------------------------------
def trata_sonar():
	global range_LF											#leitura do sonar
	global range_LM											#leitura do sonar
	global range_RM											#leitura do sonar
	global range_RF											#leitura do sonar
	global range_side_L
	global range_side_R
	global max_range_LF
	global max_range_LM
	global max_range_RM
	global max_range_RF
	global max_range_side_L
	global max_range_side_R
	desvio = 0
	divisor = 2

	var_print = OFF

	if SONAR:
		if (range_LF < max_range_LF and range_RF < max_range_RF):
			obstacle = FRENTE
			if range_LF < max_range_LF/divisor or range_RF < max_range_RF/divisor:
				desvio = DESVIO * 2
			else:
				desvio = DESVIO
		elif range_LF < max_range_LF:
			obstacle = ESQUERDA
			if range_LF < max_range_LF/divisor:
				desvio = - DESVIO * 2
			else: 
				desvio = - DESVIO 
		elif  range_RF < max_range_RF:
			obstacle = DIREITA
			if range_RF < max_range_RF/divisor:
				desvio = DESVIO * 2
			else:
				desvio = DESVIO 
		elif range_LM < max_range_LM:
			obstacle = ESQUERDA
			if range_LM < max_range_LM/divisor:
				desvio = - DESVIO * 2
			else:
				desvio = - DESVIO 
		elif range_RM < max_range_RM:
			obstacle = DIREITA
			if range_RM < max_range_RM/divisor:
				desvio = DESVIO *2
			else:
				desvio = DESVIO 
		elif range_side_L < max_range_side_L:
			obstacle = LADO_ESQUERDO
			desvio = - DESVIO/2 
		elif range_side_R < max_range_side_R:
			obstacle = LADO_DIREITO
			desvio = DESVIO/2 
		else:
			obstacle = LIVRE
	else:
		obstacle = LIVRE

	if (range_LF < COLLISION or range_LM < COLLISION  or range_RM < COLLISION or range_RF < COLLISION or range_side_L < COLLISION or range_side_R < COLLISION):
		if var_print:
			print "COLLISION!"
			print "range_LF: ", range_LF
			print "range_LM: ", range_LM
			print "range_RM: ", range_RM
			print "range_RF: ", range_RF
			print "range_side_L: ", range_side_L
			print "range_side_R: ", range_side_R
		
		

	return obstacle,desvio

#-----------------------------------------------------------------------
#FUNCAO: vel_and_angle(data)
#	Funcao chamada pelo topico de parametros desejados do veiculo (drive_parameters)
#	Define variaveis de controle: 
# 	aceleracao - throttle
# 	frenagem - Brake 
# 	Angulo das rodas: steer
#	Publica no topico "car1/prius"
#-----------------------------------------------------------------------
def vel_and_angle(data):
	global speed_TV										#chamada para a variavel global
	global count
	global car_name_TV

	var_print = OFF
	msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
	msg.header.stamp = rospy.Time.now();					#tempo atualizado da leitura
	msg.header.frame_id = "base_link";						
	msg.steer = data.angle * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS
	if (stop_condition):
		msg.steer = 0.0
	
	if var_print:
		print "----START-vel_and_angle----------"
		print "vel: ", data.velocity
		print "speed_TV: ", speed_TV
		print "data_angle: ", data.angle
		print "steer: ", data.angle * ANG_TO_RAD
	
	if (data.angle * ANG_TO_RAD) > 0.05 and REDUCED_SPEED:
		#A variavel data.velocity eh dada em km/h e speed_TV eh definida em m/s
		msg.throttle, msg.brake = calc_throttle_brake_v2 (VEL_FRENAGEM, speed_TV)	#funcao que calcul aceleracao e frenagem
	else:
		#A variavel data.velocity eh dada em km/h e speed_TV eh definida em m/s
		msg.throttle, msg.brake = calc_throttle_brake_v2 (data.velocity, speed_TV)	#funcao que calcul aceleracao e frenagem

	# if count < 100:
	# 	print "count: ",count
	# 	msg.throttle = 1.0
	# 	msg.brake = 0.0
	# else:
	# 	print "count: ",count
	# 	msg.throttle = 0.0
	#	msg.brake = 0.0
	#print "Throttle=",msg.throttle
	#print "brake=",msg.brake 
	#print "angle=",data.angle
	#print "----END-vel_and_angle----------"
	pub = rospy.Publisher(car_name_TV+'prius', Control, queue_size=10)				#topico de controle do carro
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'

#-----------------------------------------------------------------------
#FUNCAO: grava_dados()
#	Grava os dados desejados em um arquivo CSV
#-----------------------------------------------------------------------
def grava_dados_TV():

	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV
	global steering_TV
	global throttle_TV
	global brake_TV
	global old_longitude_TV
	global old_latitude_TV
	global count_TV
	global lap_TV
	global count_lap_TV
	global erro_vel
	global theta_error

	global time_base
	global elapsed_time_TV
	global old_time_TV

	global range_LF											#leitura do sonar
	global range_LM											#leitura do sonar
	global range_RM											#leitura do sonar
	global range_RF											#leitura do sonar
	global range_side_L
	global range_side_R
	global max_range_LF
	global max_range_LM
	global max_range_RM
	global max_range_RF
	global max_range_side_L
	global max_range_side_R

	#print 'GRAVA_DADOS'

	var_print = 0  # permite imprimir na funcao

	#current_time_TV = tp.time()
	current_time_TV = rospy.get_time()
	elapsed_time_TV = current_time_TV - old_time_TV

	# atualiza as leituras de posicao se maior que um limiar
	#if (abs(longitude_TV-old_longitude_TV) > 0.5 or abs(latitude_TV-old_latitude_TV) > 0.5):
	if elapsed_time_TV >= TIME_SPACE:
		old_time_TV = current_time_TV			#record the last time event
		old_longitude_TV = longitude_TV  # atualiza as ultimas leituras validas de longitude
		old_latitude_TV = latitude_TV  # atualiza as ultimas leituras validas de latitude

		if var_print:  # ultimas leituras
			print "latitude_TV: ", latitude_TV
			print "longitude_TV: ", longitude_TV
			print "lap:" , lap
			a = abs(longitude_TV-init_long_TV)
			b = abs(latitude_TV-init_lat_TV)
			print "a:", a
			print "b:", b


	if GRAVA_SONAR:
		if count_TV == 0:
			with open('/home/enio/OneDrive/Cister/ROS/CarSim/CISTER_car_drive_controller/src/image_processing/log_position_SV_'+TV_name+'.csv', mode='w') as log_position_file:
				log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				#log_writer.writerow(['count','latitude', 'longitude', 'speed_TV','speed km','heading','lap','SLF','SLM','SRM','SRF','SSL','SSR'])
				#log_writer.writerow([tp.time()-time_base, count_TV,latitude_TV, longitude_TV,speed_TV, speed_TV * 3.6, heading_TV, lap_TV, range_LF, range_LM, range_RM, range_RF, range_side_L, range_side_R])
				log_writer.writerow([rospy.get_time()-time_base, count_TV,latitude_TV, longitude_TV,speed_TV, erro_vel, heading_TV,theta_error,theta_error*ANG_TO_RAD,throttle_TV,brake_TV, lap_TV, range_LF, range_LM, range_RM, range_RF, range_side_L, range_side_R])
			count_TV = count_TV + 1
		else :
			with open('/home/enio/OneDrive/Cister/ROS/CarSim/CISTER_car_drive_controller/src/image_processing/log_position_SV_'+TV_name+'.csv', mode='a') as log_position_file:
				log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				log_writer.writerow([rospy.get_time()-time_base, count_TV,latitude_TV, longitude_TV, speed_TV,erro_vel,heading_TV,theta_error,theta_error*ANG_TO_RAD,throttle_TV,brake_TV,lap_TV, range_LF, range_LM, range_RM, range_RF, range_side_L, range_side_R])
			count_TV = count_TV + 1
	else:
		if count_TV == 0:
			with open('/home/enio/OneDrive/Cister/ROS/CarSim/CISTER_car_drive_controller/src/image_processing/log_position_SV_'+TV_name+'.csv', mode='w') as log_position_file:
				log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				#log_writer.writerow(['time','count','latitude_TV', 'longitude_TV', 'speed_TV','speed km','heading_TV','lap'])
				log_writer.writerow([rospy.get_time()-time_base, count_TV,latitude_TV, longitude_TV,speed_TV, erro_vel, heading_TV,theta_error,theta_error*ANG_TO_RAD,throttle_TV,brake_TV, lap_TV])
			count_TV = count_TV + 1
		else :
			with open('/home/enio/OneDrive/Cister/ROS/CarSim/CISTER_car_drive_controller/src/image_processing/log_position_SV_'+TV_name+'.csv', mode='a') as log_position_file:
				log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				log_writer.writerow([rospy.get_time()-time_base, count_TV,latitude_TV, longitude_TV, speed_TV,erro_vel,heading_TV,theta_error,theta_error*ANG_TO_RAD,throttle_TV,brake_TV,lap_TV])
			count_TV = count_TV + 1


#-----------------------------------------------------------------------
#FUNCAO: car_info(data)
#	Funcao chamada pelo topico de dados atuais do veiculo ('/car1/carINFO')
#-----------------------------------------------------------------------
def car_info(data):
	global speed_TV										#chamada para a variavel global
	global longitude_TV
	global latitude_TV
	global heading_TV
	global throttle_TV
	global brake_TV
	global count_lap_TV
	global lap_TV
	global VEL_CRUZEIRO
	global VEL_FRENAGEM
	global TARGET_SPEED
	speed_TV = data.speed_speedValue						#salva o valor da velocidade no instante
	latitude_TV = data.latitude
	longitude_TV = data.longitude
	heading_TV = data.heading_headingValue
	drive_dir_TV = data.driveDirection
	steer_TV = data.steeringWheelAngle_steeringWheelAngleValue
	throttle_TV = data.gasPedalPercent_Value
	brake_TV = data.brakePedalPercent_Value

	#if (abs(longitude_TV-LAP_LON) < 1 and abs(latitude_TV-LAP_LAT) < 2) or (abs(longitude_TV-LAP_LON_2) < 1 and abs(latitude_TV-LAP_LAT_2) < 2) or (abs(longitude_TV-LAP_LON_3) < 1 and abs(latitude_TV-LAP_LAT_3) < 2):  	# conta o numero de voltas
	for i in range(len(LAP_LAT_V)):
		if (abs(longitude_TV-LAP_LAT_V[i][1]) < 1 and abs(latitude_TV-LAP_LAT_V[i][0]) < 2):
			if count_lap_TV == i:  												# so adiciona voltas se count_lap for igual a zero
				lap_TV = lap_TV + 1  														# incrementa o numero de voltas
				print "lap_TV:" , lap_TV
				count_lap_TV = count_lap_TV + 1  											# incrementa o indicador de posicao
				print "count_lap_TV: ", count_lap_TV
			#else:
				#count_lap_TV = 1 												# indica que o veiculo ja passou do ponto inicial
				#break
		#else:
			#count_lap_TV = 0
	if lap_TV == 1:
		TARGET_SPEED = TARGET_SPEED_2
		VEL_CRUZEIRO = VEL_CRUZEIRO_2
		VEL_FRENAGEM = VEL_FRENAGEM_2
	elif lap_TV == 2:
		TARGET_SPEED = TARGET_SPEED_3
		VEL_CRUZEIRO = VEL_CRUZEIRO_3
		VEL_FRENAGEM = VEL_FRENAGEM_3
	elif lap_TV == 3:
		TARGET_SPEED = TARGET_SPEED_4
		VEL_CRUZEIRO = VEL_CRUZEIRO_4
		VEL_FRENAGEM = VEL_FRENAGEM_4
	elif lap_TV == 4:
		TARGET_SPEED = TARGET_SPEED_5
		VEL_CRUZEIRO = VEL_CRUZEIRO_5
		VEL_FRENAGEM = VEL_FRENAGEM_5
	elif lap_TV == 5:
		TARGET_SPEED = TARGET_SPEED_6
		VEL_CRUZEIRO = VEL_CRUZEIRO_6
		VEL_FRENAGEM = VEL_FRENAGEM_6
	elif lap_TV == 6:
		TARGET_SPEED = TARGET_SPEED_7
		VEL_CRUZEIRO = VEL_CRUZEIRO_7
		VEL_FRENAGEM = VEL_FRENAGEM_7
	elif lap_TV == 7:
		TARGET_SPEED = TARGET_SPEED_8
		VEL_CRUZEIRO = VEL_CRUZEIRO_8
		VEL_FRENAGEM = VEL_FRENAGEM_8
	else:
		TARGET_SPEED = TARGET_SPEED_1
		VEL_CRUZEIRO = VEL_CRUZEIRO_1
		VEL_FRENAGEM = VEL_FRENAGEM_1

	grava_dados_TV()



#-----------------------------------------------------------------------
#FUNCAO: vehicle_control(data)
#	Le os dados de posicao da camera ('X_Y')
#-----------------------------------------------------------------------
def vehicle_control(data):
	global prev_error_steer 
	global prev_error_vel
	global error_steer
	global erro_vel
	global angle
	global velocity
	global velocity_number
	global prev_theta
	global count											#numero de iteracoes
	global time1											#tempo entre execucoes
	global steer_integral									#acumulador do erro
	global steer_deriv

	global theta_error

	global range_LF											#leitura do sonar
	global range_LM											#leitura do sonar
	global range_RM											#leitura do sonar
	global range_RF											#leitura do sonar

	global VEL_CRUZEIRO
	global VEL_FRENAGEM

	global car_name_TV
	var_print = OFF
	
	if var_print:
		print "-----------Vehicle CONTR0L ----------"	
		print "count", count
	obstacle,desvio = trata_sonar()								#calcula se ha obstaculo a frente
	#print 'Obstacle: ', obstacle

	#define o dt--------------------------------------------------------------------------------------------------------------------------------------
	time_now = tp.time()									#armazena o tempo atual
	if (count == 0):
		dt = 0.0											#inicializa a variavel
	else:	
		dt = time_now - time1								#tempo entre iteracoes
	time1 = time_now										#armazena o tempo atual para a proxima iteracao

	#armazenando os dados lidos do topico--------------------------------------------------------------------------------------------------------------------------------------
	x2 = data.X2												#posicao X2 da linha detectada
	y2 = data.Y2												#posicao Y2 da linha detectada
	x1 = data.X1
	#x_mean = (x2+x1)/2
	x_mean = x2
	slope = data.slope
	if var_print:
		print "x2: ", x2
		print "x1: ", x1
		print "x_mean: ", x_mean
		print "slope: ", slope


	#definindo o erro de posicao do veiculo em relacao a linha
	error_steer = (x_mean-400)/400								#posicao X da linha em relacao ao centro da imagem 800 pixels
	#error_steer = x_mean-400								#posicao X da linha em relacao ao centro da imagem 800 pixels
	if var_print:
		print "Error: ",error_steer
	
	#sistema de controle de velocidade--------------------------------------------------------------------------------------------------------------------------------------
	#print "Count: ",count
	velocity = VEL_CRUZEIRO									#atualiza velocidade do veiculo
	count = count+1											#conta numero de iteracoes
	if var_print:
		print "vel:",velocity

	#Controle de tempo de resposta--------------------------------------------------------------------------------------------------------------------------------------
	if abs(slope) <= MIN_SLOPE and x2-x1 > 30:
		dist_corr = velocity * KMH_TO_MS * 0.5
	else:
		dist_corr = velocity * KMH_TO_MS * 1						#distancia a ser percorrida em metros em 1 s
	if var_print:
		print "dist_corr:",dist_corr


	#define o erro em termos de angulo--------------------------------------------------------------------------------------------------------------------------------------
	if (velocity != ZERO):
		dist_ang = error_steer/dist_corr						#angulo do erro em relacao a reta detectada
	else:
		dist_ang = ZERO
	#print "cat op sobre hip: ", error_steer/dist_corr
	if var_print:
		print "dist_ang:",dist_ang

	if(abs(dist_ang)<=1 and velocity != ZERO):
		if var_print:
			print "Correcao Normal"
		theta_error = np.arcsin(error_steer/dist_corr) * RAD_TO_ANGLE	#correcao normal
	elif (velocity != ZERO):
		theta_error = np.arcsin((error_steer/abs(error_steer))/(dist_corr/abs(dist_corr))) * RAD_TO_ANGLE	#se a distancia inicial for pequena para o erro
		if var_print:
			print "Distancia Pequena"
	else:
		theta_error = ZERO					#caso nao exista uma linha detectada
	
	if var_print:
		print "theta_error: ", theta_error

	control_error_steer = calc_PID( theta_error, dt, slope, x2, x1)
	#print 'theta_error: ', theta_error,' Control_error_steer: ',control_error_steer

	prev_theta = theta_error							#atualiza o valor do erro
	
	if (obstacle == LIVRE):								# se nao houver obstaculo a frente
		angle = control_error_steer#*ANG_TO_RAD				#atualiza o angulo de controle
		#print 'LIVRE'
	else:												#atuacao em caso de obstaculo a frente
		angle = desvio
		prev_theta = 0
		steer_integral = ZERO
		velocity = VEL_FRENAGEM
		#print 'OBSTACLE'



	if angle > MAX_STEER:								#limita o angulo maximo
		angle = MAX_STEER
	if angle < -MAX_STEER:
		angle = -MAX_STEER
	#print "angle:",angle
	#--------------------------
	if var_print:
		print "-----------END Vehicle CONTROL ----------"

	#prepara msg para publicacao------------------------------------------------------------------------------------------------------------------------------
	msg = drive_param()
	#print "velocity: ", velocity
	msg.velocity = velocity
	msg.angle = angle
	pub_2 = rospy.Publisher(car_name_TV+'drive_parameters', drive_param, queue_size=1)	#parametros de velocidade do veiculo
	pub_2.publish(msg)

	#msg_e = error_control()
	#msg_e.error_steer = error_steer
	#msg_e.control_error_steer = control_error_steer
	#msg_e.steer_integral = steer_integral
	#msg_e.steer_deriv = steer_deriv
	#msg_e.status = status
	#pub_3.publish(msg_e)

#-----------------------------------------------------------------------
#FUNCAO: listener()
#	inicializa o no
#	assina os topicos de leitura desejados
#-----------------------------------------------------------------------
def listener():
	global car_name_TV
	global TV_name

	rospy.init_node('PriusHybridPlugin', anonymous=True)			#inicializa o no
	rospy.Subscriber(car_name_TV +'X_Y', coords, vehicle_control)				#subscreve no topico e chama a funcao vehicle_control
	rospy.Subscriber(car_name_TV +'drive_parameters', drive_param, vel_and_angle)#subscreve no topico e chama a funcao vel_and_angle
	rospy.Subscriber(car_name_TV +'carINFO',CAM_simplified,car_info)		#subscreve no topico e chama a funcao car_info
	rospy.Subscriber(car_name_TV +'front_sonar_left_far_range',Sonar,sonar_front_LF)#subscreve ao topicoi e chama a funcao sonar_LF
	rospy.Subscriber(car_name_TV +'front_sonar_left_middle_range',Sonar,sonar_front_LM)#subscreve ao topicoi e chama a funcao sonar_LM
	rospy.Subscriber(car_name_TV +'front_sonar_right_middle_range',Sonar,sonar_front_RM)#subscreve ao topicoi e chama a funcao sonar_RM
	rospy.Subscriber(car_name_TV +'front_sonar_right_far_range',Sonar,sonar_front_RF)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber(car_name_TV +'side_sonar_left_front_range',Sonar,sonar_side_L)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber(car_name_TV +'side_sonar_right_front_range',Sonar,sonar_side_R)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.spin()													#mantem o listener aberto

if __name__ == '__main__':
	param = sys.argv[1:]
	if len(param) != 0:
		TV_name =  param[0]
		car_name_TV = "/" + param[0] +"/"
		print (car_name_TV)
		publication_topic = "/" + param[0] + "/X_Y"
	else:
		car_name_TV = "/" + "car1" +"/"
		print (car_name_TV)
		publication_topic = "/" + "car1" + "/X_Y"
	if len(param) == 2: 
		VEL_CRUZEIRO = int(param[1])
		VEL_FRENAGEM = int(param[1])
		TARGET_SPEED = VEL_CRUZEIRO/3.6
		print ("aqui?")

	#time_base = tp.time()
	time_base = 0.0

	print "Simulation_Connector"
	if REDUCED_SPEED:
		print "REDUCED SPEED IN CURVES"
	listener()


