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
#v6
#   Corrigindo erro no circulo trigonometrico
#v7 
#   Adjusting the PID Distance
#   Adding the Normal and Lost Mode
#v8 
#   Adding the bearing (FIXED)
#   Adjusting the angle following (theta error)
#   adjustin the angles of the vehicles given the position of the road
#   Removing Normal and Lost mode - there is only one mode to find the most near position in front of the vehicle
#v91
#   Adjusting PID values
#   Removing "/dt" from PID calculation
#v92
#   Writting PID and Theta errors to error_control
#v93
#   changing the calculum from the maximum distance
#   Bearing control variable added (ON-OFF)
#v94
#   Solving a problem in Theta Error
#v100
#   Adding the error from the previous car
#v101
#   Adjusting brake situations
#   Adjusting initial speed
#v102
#	PID adjusts (40 and 60 Km/h)
#	More laps
#	Bearing modification (inverse SEN and COS)
#	No bearing when LOST
#v103
#	Cam modification for message count
#	Change in TV_position_vector tosupport the message count
#-----------------------------------------------------------------------
import rospy
import sys												  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
#from image_processing.msg import AckermannDriveStamped
from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math												 #library for math functions
import numpy as np
import time as tp
import csv
import numpy as np
#------------------------------------------------------------------------

#Publication Topic

#------------------------------------------------------------------------
#global variables  
#data from the TV --------------------------------------------
car_name_TV = 0										  #TV name
longitude_TV = 0.0									   #longitude
latitude_TV = 0.0										#latitude 
heading_TV = 0.0										 #heading (angulacao global do veiculo)
speed_TV = 0.0											 #vehicle speed (m/s)
steering_TV = 0.0										#steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_TV = 0.0										#percentual do acelerador (0 - 1)
brake_TV = 0.0										   #percentual do freio (0 - 1)
speed_TV_reference = 0.0								 #reference speed for the maximum_distance
speed_TV_OLD = 0.0
heading_TV_OLD = 0.0										 #Used to define if TV is changing direction
message_id_TV = 0										#message ID
message_id_TV_OLD = 99

#data from the SV --------------------------------------------
longitude_SV = 0.0									   #longitude
latitude_SV = 0.0										#latitude 
heading_SV = 0.0										 #heading (angulacao global do veiculo)
speed_SV = 0.0											#vehicle speed (m/s)
steering_SV = 0.0										#steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0										#percentual do acelerador (0 - 1)
brake_SV = 0.0										   #percentual do freio (0 - 1)
message_id_SV = 0										#message ID

#variaveis de controle --------------------------------------------
kp_dis = 2.0		#0.9								#Distance Proportional Controller
ki_dis = 0.005	   #0.01							  #Distance Integral Controller
kd_dis = 1.0		#0.3								#Distance Derivative Controller

dis_integral = 0.0									  #Distance Integral Acumulator
dist_deriv = 0.0										#Distance derivative data (does not need to be global)
erro_dis_old = 0.0									  #Distance previous error

kp_theta = 2.3		#Work: 2.0 	  	#3.0	5.0		#6.0						#Angle Proportional Controller
ki_theta = 0.01		#Work: 0.005	#0.005	0.005	#0.05					  #Angle Integral Controller
kd_theta = 1.0	  	#Work: 1.0		#1.0	1.0		#3.0						#Angle Derivative Controller

theta_integral = 0.0									#Angle Integral Acumulator
theta_deriv = 0.0									   #Angle derivative data (does not need to be global)
theta_erro_old = 0.0									#Angle previous error
theta_error = 0.0

#variaveis gerais -------------------------------------------------
TV_topic = 0											#topico do TV
SV_topic = 0											#topico do SV
TV_control_topic = 0									#control topic from local leader
publication_error_topic = 0							 #topico a publicar dados de controle do veiculo
publication_topic = 0								   #topico a publicar dados do veiculo

incremental_controller = 0							  #indicates that the incremental controller is ON or OFF

TV_PID_error = 0										#Distance error from TV
TV_THETA_error = 0									  #Theta error from TV

time_distance_control = 0.0							 #time of last distance control interaction 
time_direction_control = 0.0							#time of last direction control interaction 

TV_position_vector = np.empty([1,6])					#vector to store TV positions to be followed
														#[latitude_TV, longitude_TV, heading_TV, speed_TV, tp.time(), message_id]
TV_error_vector = np.empty([1,2])					   #vector to store TV errors to be added to SV controller

pid_adjust_old = 0.0 									#in case of interactions smaller then dt

gen_cam_id = 0				#used to generate CAM_ID for eachmessage. Use with RXNetwork, remove with RXNetwork_OMNET
TV_used_ID = 0				#
#debug tools--------------------OFF
LOST = 0
NORMAL = 1
count = 0

#defines------------------------------------------------------------
ON = 1						  #Debug mode
OFF = 0						 #Debug mode

MAX_SPEED = 100.0/3.6			#velocidade maxima do veiculo (m/s)
#MAX_SPEED = 60.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 80.0				#maxima angulacao da roda em graus
MIN_STEER = -80.0				#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6					#converte de m/s para Km/h
EDGE_LOW_SPEED	= 0.05			#ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
EDGE_ANGLE = 0.005
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#

MINIMUM_DISTANCE = 5.5		#5.5			#Safety distance - m
SAFETY_TIME = 0.5			#0.5			   #safety time - s
GPS_RANGE = 3.0				 #range to used in comparison between th SV position and the historical position of TV
MAX_ERRO_INTEGRADOR_DIS = 50	#UP limit for the integrator in distance
MAX_ERRO_INTEGRADOR_THETA = 0.1  #UP limit for the integrator in theta
BEARING_ANGLE_LIM = 0.15		#MIN limit for bearing check (angle)
#BEARING_ANGLE_LIM = 0.07		#MIN limit for bearing check (angle)
#BEARING_ANGLE_LIM = 0.2618		#MIN limit for bearing check (angle)
BEARING_DIST_LIM = 1.0		#MIN limit for bearing check (DIST)

BEARING_CONTROL = ON			#activate or desactivate the BEARING conbtrol
INITIAL_SPEED = ON			  #define a initial speed
FULL_BRAKE = ON				#enable the full brake when the TV Stops

#---------just one lap
TARGET_SPEED = 14.00			#target speed (approx 50KM/h)
#---------More then one lap
TARGET_SPEED_1 = 14.00			#target speed (approx 50KM/h)
TARGET_SPEED_2 = 12.00			#target speed (approx 40KM/h)
TARGET_SPEED_3 = 16.00			#target speed (approx 60KM/h)


ANGLE_FRONT = math.pi/4		 #angle to determine if the historical point is in fron or not
ANGLE_ADJUST = math.pi/2		#adjust provided by the difference between the trigonometric circle and the road

ZERO = 0

MINIMUM_DT = 0.001			   #time between control actions
#MINIMUM_DT = 0.03			   #time between control actions


#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(desired_speed, real_speed)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(desired_speed, real_speed):
	#print "desired speed: ",desired_speed
	standard_speed = 0		  #while the car does not achieve defined speed
	erro_vel = desired_speed  - real_speed			#calcula o erro de velocidade -> "/3.6" converte para m/s
	erro_vel_norm = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)								#acao de controle
	accel_control = max(accel_control,-1.0)								#acao de controle
	if(abs(accel_control) < EDGE_LOW_SPEED and desired_speed < EDGE_LOW_SPEED):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
		#print "brake"
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem
		if real_speed >= TARGET_SPEED:
			standard_speed = 1
		if standard_speed == 0 and INITIAL_SPEED:
			aceleracao = 1.0
		#print "usual"
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: distance_control(d_error)
#   Control the distance between the TV and the SV using the speed of the SV	
#   v_TV -> TV speed
#   v_SV -> SV Speed
#   d_error -> Error between the desired distance and the real distance
#   Return: control_vel (Data responsible for the new desired speed in m/s)
#-----------------------------------------------------------------------
def distance_control (d_error):
	#Global variables-----------------
	global dis_integral
	global dist_deriv
	global erro_dis_old
	
	global time_distance_control							#time of last distance control interaction
	global pid_adjust_old									#in case of interactions smaller then dt

	debug = OFF											  #defines if the text will be printed
	#Calcule dt------------------------------------------------------------
	#used to determine the time between interactions from the program
	time_now = tp.time()									#saves te time of the system
	dt = time_now - time_distance_control					#time between interactions
	if (dt >= MINIMUM_DT):								  #time between control actions
	#if (1):								  #time between control actions
		time_distance_control = time_now					#save the time for next interaction

		#determines the integrator component
		if (abs(dis_integral) >= MAX_ERRO_INTEGRADOR_DIS or dt>10):
			dis_integral = ZERO
		else:
			dis_integral = dis_integral + d_error * dt
		
		#derivative value
		if (dt < 1):
			dist_deriv = (erro_dis_old - d_error)
		else:
			dist_deriv = (erro_dis_old - d_error)
		erro_dis_old = d_error

		#calcula PID de distancia
		PID_dis_adjust = kp_dis * d_error + ki_dis * dis_integral + kd_dis * dist_deriv
		pid_adjust_old = PID_dis_adjust
	else:
		PID_dis_adjust = pid_adjust_old
		print "Time interval < dt (DIST)"
	if debug:
		print ("PID_Dis_adjust: ", PID_dis_adjust)
	return PID_dis_adjust
	
#-----------------------------------------------------------------------
#FUNCAO: bearing_calculator()
#	calculates the bearing between SV and the desired position from TV
#   return bearing angle
#-----------------------------------------------------------------------
def bearing_calculator(d_lat, d_long, lat_TV, long_TV, head_TV, head_SV):
	
	#global data
	global latitude_SV					  #SV latitude
	global longitude_SV					 #SV longitude

	bearing_angle = 0.0								 #bearing
	debug = OFF							 #modo de debug

	# if abs(head_TV) < math.pi/4 or abs(head_TV) >= 3*math.pi/4:
	# 	bearing_angle = math.atan2 (d_long, d_lat)		  #bearing calculation
	# else:
	# 	bearing_angle = math.atan2 (d_lat, d_long)		  #bearing calculation

	bearing_angle = math.atan2 (d_long,d_lat)		  #bearing calculation

	if (bearing_angle < 0.0 and head_SV > math.pi/4):   #bearing adjust if bearing is negative
		bearing_angle = bearing_angle + 2 * math.pi

	if debug:
	#if abs(bearing_angle) > 0.15: 
		print "bearing calculator"
		print "TV: ", lat_TV, long_TV, head_TV
		print "SV: ", latitude_SV, longitude_SV, head_SV
		print "d_long: ", d_long
		print "head_TV: "
		print "bearing_angle: ", bearing_angle
	

	return bearing_angle

#-----------------------------------------------------------------------
#FUNCAO: direction_control(heading_TV, heading_SV)
#   Control the direction of the SV using the heading of the TV	
#   h_TV -> TV speed
#   h_SV -> SV Speed
#   Return: theta_control 
#-----------------------------------------------------------------------
def direction_control (lat_TV, long_TV, h_TV, h_SV):
	#Global variables-----------------
	global theta_integral								   #Angle Integral Acumulator
	global theta_deriv									  #Angle Derivative Acumulator
	global theta_erro_old								   #Angle previous error
	global theta_error									  #Angle current error
	global time_direction_control						   #time of last direction control interaction
	global latitude_SV									  #SV latitude
	global longitude_SV									 #SV Longitude
	global latitude_TV										#TV latitude
	global longitude_TV										#TV longitude
	global heading_TV_OLD

	global LOST

	debug = OFF											 #modo debug
	#debug2 = ON											 #modo debug
	bearing_angle = 0.0									 #bearing Angle
	
	#Calcule dt------------------------------------------------------------
	#used to determine the time between interactions from the program
	time_now = tp.time()									#armazena o tempo atual
	dt = time_now - time_direction_control					#tempo entre iteracoes
	if (dt >= MINIMUM_DT):								  #act only after new controller variables
		time_direction_control = time_now					#armazena o tempo atual para a proxima iteracao

		if debug:
			print "------------------DC--------------------------------"
			print "h_TV: ", h_TV
			print "h_SV: ", h_SV

		#------------------------------------------------------------------------
		#adjusting the angles given the rotation between the circle and the road (pi/2)
		#------------------------------------------------------------------------
		h_TV = h_TV - ANGLE_ADJUST
		h_SV = h_SV - ANGLE_ADJUST

		#-------------------------------------------------------------------------
		#solving the problemas with diferent positions in the trigonometric circle:
		#ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
		#to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
		# -------------------------------------------------------------------------  
		if (h_TV < - math.pi/4):				#adjusting h_tv
			theta_TV = 2*math.pi + h_TV		 #adding 2pi in order to change for positive
		else:
			theta_TV = h_TV					 #if not, keep the angle
		
		if (h_SV < - math.pi/4):				#adjusting h_sv
			theta_SV = 2*math.pi + h_SV		 #adding 2pi in order to change for positive
		else:
			theta_SV = h_SV					 #if not, keep the angle
		
	   
		theta_error = theta_TV - theta_SV	   #calculate theta_error
		
		if debug:
		#if abs(theta_error) > 0.15:
			print "Theta_TV: ", theta_TV
			print "Theta_SV: ", theta_SV
			print "theta_error: ", theta_error

		#find the distance between the current SV point and the evaluated TV point
		#d_lat = lat_TV - latitude_SV			#calculates the diference in latitude
		#d_long = long_TV - longitude_SV		 #calculates the diference in longitude

		#d_lat = latitude_TV - latitude_SV			#calculates the diference in latitude
		#d_long = longitude_TV - longitude_SV		 #calculates the diference in longitude

		#-------------------------------------------------------------------------
		# IF the TV is not changing it's direction, the SV compares it's own postion with
		# the TV CURRENT position, to define the bearing.
		# ELSE, it compares the SV position with the EVALUATED position of TV
		# -------------------------------------------------------------------------  
		if (abs(heading_TV_OLD - heading_TV)<0.01):		#IF TV is not changing it's direction	
			d_lat = latitude_TV - latitude_SV			#calculates the diference in latitude (using the CURRENT TV position)
			d_long = longitude_TV - longitude_SV		 #calculates the diference in longitude (using the CURRENT TV position)
		else:										#IF TV is changing it's direction	(to avoid the "cutting corner effect")
			d_lat = lat_TV - latitude_SV			#calculates the diference in latitude (using the EVALUATED TV position)
			d_long = long_TV - longitude_SV		 #calculates the diference in longitude  (using the EVALUATED TV position)



		dist = (d_lat**2 + d_long**2)**0.5	  #calculates the distance

		#---------------------------------------------------------------------------
		#   Evaluate if the current position of SV is displaced from TV even if the angle theta_error is small
		#   Only checked if theta_error is small enough and if the distance between the evaluated points are not too small
		#   If the distance is very small, the atan2 function does not work properly, generating errors
		#---------------------------------------------------------------------------
		
		if (abs(theta_error) < BEARING_ANGLE_LIM and dist > BEARING_DIST_LIM and BEARING_CONTROL and LOST == 0):
			
			bearing_angle = bearing_calculator(d_lat, d_long, lat_TV, long_TV, theta_TV, theta_SV)  #calls the bearing angle function
			
			diff = abs(theta_TV - bearing_angle)						#the difference between the bearing angle and the TV angle
			if (diff > math.pi/2):
				bearing_angle = bearing_calculator(d_long, d_lat, lat_TV, long_TV, theta_TV, theta_SV)  #calls the bearing angle function
				diff = abs(theta_TV - bearing_angle)						#the difference between the bearing angle and the TV angle
				if (diff > math.pi/2):
					diff = 0.0

			theta_error_orig = 	theta_error		
			if (bearing_angle > theta_SV):							  #determine if the bearing adjust is positive or negative
				theta_error = theta_TV - theta_SV + diff				#is positive if bearing angle is bigger then the SV angle
			else:
				theta_error = theta_TV - theta_SV - diff				#is negative if bearing angle is bigger then the SV angle
			
			if debug:
			#if abs(theta_error) > 0.15:
				 print "--------------------------------"
				 print "bearing_angle: ", bearing_angle
				 print "diff: ", diff
				 print "Theta_error_orig: ", theta_error_orig
				 print "Theta_error (A): ", theta_error
				 print "h_TV and h_SV: ", h_TV, h_SV
				 print "theta_TV, theta_SV:  ", theta_TV, theta_SV
				 print "d_lat, d_long: ", d_lat, d_long
				 print "Lat_TV, long_TV: ", lat_TV, long_TV 
				 print "Lat_SV, long_SV: ", latitude_SV, longitude_SV

		#---------------------------------------------------------------------------
		#   Sometimes, SV angle and TV angle does not match the trigonometric adjust conditions at the same time
		#   In this situations, the difference between those angles is very big and out the trigonometric circle
		#   To adjust this, if the error outpass PI, the previous adjust is desconsidered and the simple error between the angles is considered
		#---------------------------------------------------------------------------
		if (abs(theta_error) > math.pi):					
			theta_error = h_TV - h_SV
			#if debug or abs(theta_error) > 0.25:
			if debug:
				print "Theta_error: ", theta_error
				print "h_TV and h_SV: ", h_TV, h_SV

		#---------------------------------------------------------------------------
		#   PID controller for the Angular Position
		#---------------------------------------------------------------------------
		if (abs(theta_error) >= EDGE_ANGLE):							#If theta_error is significative
			if debug:
				print "ANGLE_DIFERENTE"
			
			#Update the angular integrator value
			if (abs(theta_integral) >= MAX_ERRO_INTEGRADOR_THETA or dt>10):
				theta_integral = ZERO	
			else:
				theta_integral = theta_integral + theta_error * dt
			
			#Update the angular derivative value
			#theta_deriv = (theta_erro_old - theta_error)/dt
			theta_deriv = (theta_erro_old - theta_error)
			#if debug or abs(theta_error) > 0.25:
			#	print "theta_error_old: ", theta_erro_old
			#	print "theta_error: ", theta_error
			theta_erro_old = theta_error

			#PID Action
			theta_control = ( kp_theta * theta_error + ki_theta * theta_integral + kd_theta * theta_deriv)
			
			if debug:
			#if abs(theta_error) > 0.15:
				print "theta_control (B): ", theta_control
				print "Theta_error (D): ", theta_error
				#print "Integral: ", theta_integral
				#print "Deriv:", theta_deriv
				#print "dt: ",dt
			#---------------------------------------------------------------------------
			#   The maximum steering of the car is 20 degrees (0.34 radians)
			#---------------------------------------------------------------------------
			if (theta_control > MAX_STEER * ANG_TO_RAD):		#adjust the maximum control action
				if debug:
					print "Theta_error: ", theta_error
					print "theta_control (D): ", theta_control
					print "Integral: ", theta_integral
					print "Deriv:", theta_deriv
					print "-----------"
				theta_control = MAX_STEER * ANG_TO_RAD
				
			elif (theta_control < - MAX_STEER * ANG_TO_RAD):	#adjust the maximum control action
				if debug:
					print "Theta_error: ", theta_error
					print "theta_control (E): ", theta_control
					print "Integral: ", theta_integral
					print "Deriv:", theta_deriv
					print "-----------"
				theta_control = - MAX_STEER * ANG_TO_RAD
				
			if debug:
			#if abs(theta_error) > 0.15:
				print "theta_control (C): ", theta_control
		else:													   #if theta_error is not significative
			if debug:
				print "ANGLE_IGUAL"
			theta_error = ZERO								  #clear theta_error									 
			theta_control = ZERO								#clear theta control
			theta_integral = ZERO							   #clear theta integral
			theta_erro_old = ZERO							   #clear previous theta_error


		return theta_control
	else:													   #if theres not new values from the sensors controller variables
		print "Time interval < dt (theta)"
		return 0.0

#-----------------------------------------------------------------------
#FUNCAO: compare_lost_position():
#This function compares the SV position with the historical position from TV
#Uses the TV vector to find the nearest position in front of SV, considering the current position of SV and
#historical positions from TV
#-----------------------------------------------------------------------
def compare_lost_position():
	global TV_position_vector						   #vector to store TV positions to be followed
	global longitude_SV								 #SV Longitude
	global latitude_SV								  #SV Latitude
	global heading_SV								   #SV heading
	global mode
	global count

	global TV_error_vector							  #TV errors vector
	global TV_used_ID
	
	dist = 0											#distance between the current SV position and TV vector
	dist_min = 999									  #minimun distance value
	pos = 0											 #position of the smallest distance in the vector

	dist_lost = 0									   #if the position is not in front of SV, stores the current distance
	dist_lost_min = 9999.0								 #minimun distance value
	pos_lost = 0										#position of the smallest distance in the vector

	debug = OFF

	TV_pid_out = 0.0
	TV_theta_out = 0.0

	heading_out = 0.0

	#------------------------------------------------------------------------
	#adjusting the angles given the rotation between the circle and the road (pi/2)
	#------------------------------------------------------------------------
	head_SV_adjusted = heading_SV - ANGLE_ADJUST
	
	#-------------------------------------------------------------------------
	#solving the problemas with diferent positions in the trigonometric circle:
	#ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
	#to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
	# -------------------------------------------------------------------------  
	if (head_SV_adjusted < -math.pi/4):
		head_SV_adjusted = head_SV_adjusted + 2 * math.pi

	theta_teste = 0.0
	diff_teste = 0
	theta_SV_orig = 0
	theta_SV_orig_salvo = 0.0
	d_theta_salvo = 0
	d_time = 0

	i = 0
	#print "BEFORE FOR"
	#-------------------------------------------------------------------------
	#   Compares the historical position from TV with current position of SV		
	#-------------------------------------------------------------------------	
	for i in range(TV_position_vector.shape[0]):			#for each position in TV_position_vector
		
		if debug:
			print "i: ", i
			print "TV_position_vector: ", TV_position_vector[i,1], TV_position_vector[i,0], TV_position_vector[i,2], TV_position_vector[i,3], TV_position_vector[i,5]
			print "SV:				 ", longitude_SV, latitude_SV, heading_SV, speed_SV
		
		if (TV_position_vector[i,5] <> TV_used_ID):
			d_lat = TV_position_vector[i,0] - latitude_SV			   #calculates the distance in latitude (x)
			d_long = TV_position_vector[i,1] - longitude_SV			 #calculates the distance in longitude (y)
			theta_SV_TV = math.atan2(d_long, d_lat)					 #calculate the angle between the current SV position and the evaluated position of TV
			d_theta = abs(TV_position_vector[i,2]- ANGLE_ADJUST - head_SV_adjusted)

			#-------------------------------------------------------------------------
			#solving the problemas with diferent positions in the trigonometric circle:
			#ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
			#to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
			# -------------------------------------------------------------------------  
			theta_SV_orig = theta_SV_TV
			if (theta_SV_TV < 0.0 and head_SV_adjusted > math.pi/4):
				theta_SV_TV = theta_SV_TV + 2 * math.pi
			
			diff_theta = head_SV_adjusted - theta_SV_TV	 #calculates the diff between the current SV angle and the angle between TV and SV

			#if (abs(diff_theta) < ANGLE_FRONT):			 #if diff is to big, the TV position is not in front of SV
			if (abs(diff_theta) < ANGLE_FRONT or abs(diff_theta) > (math.pi - ANGLE_FRONT) or d_theta < math.pi/8):			 #if diff is to big, the TV position is not in front of SV
				dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
				if debug:
					print "VALOR A FRENTE"
			else:										   #if the evaluated TV point is not in front of SV, calculate the distance to use in a "lost mode"
				if debug:
					print "VALOR NAO A FRENTE"
				dist_lost = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
				#print "--------BEFORE IF------------"
				#print "dist_lost: ", dist_lost, " dist_lost_min: ", dist_lost_min
				if dist_lost_min > dist_lost and dist_lost > 0.0:
					#print "--------INSIDE IF------------"
					#print "dist_lost: ", dist_lost, " dist_lost_min: ", dist_lost_min
					pos_lost = i
					dist_lost_min = dist_lost
					theta_SV_orig_salvo = theta_SV_orig
					theta_teste = theta_SV_TV
					diff_teste = diff_theta
					d_theta_salvo = d_theta
					d_time = tp.time() - TV_position_vector[i,4]
				#else:
				#	print "--------------------------------"
				#	print "dist_lost: ", dist_lost, " dist_lost_min: ", dist_lost_min


			if debug:
				print "Dist: ", dist
			#--------------------------------------------------------------------------
			#If dist is smaller then the previous saved value in dist_min, update dist_min
			#However, if dist min is smaller then current dist, breaks the repetition, keeping this position for the tests
			#If there are no positions in the vector that are in front of SV, uses the small distance between SV and the full vector
			#--------------------------------------------------------------------------	
			if dist_min > dist and dist > 0.0:
				pos = i
				dist_min = dist			 #else, update dist_min value
				mode = NORMAL
				TV_used_ID = TV_position_vector[pos,5]
			elif dist_min < dist:
				mode = NORMAL
				break
			elif (i == (TV_position_vector.shape[0] - 1)):
				pos = pos_lost
				TV_used_ID = TV_position_vector[pos,5]
				mode = LOST
		#else:
		#	print "TV_position_vector[i,5] == TV_used_ID" 
	
	#print "TV:		  ", TV_position_vector[pos,0], TV_position_vector[pos,1] 
	#print "Current SV:  ", latitude_SV, longitude_SV

	heading_out = TV_position_vector[pos,2]
	if incremental_controller:
		TV_pid_out = TV_error_vector[pos,0]
		TV_theta_out = TV_error_vector[pos,1]
	if mode == LOST:
		speed_out = TV_position_vector[pos,3]
		#speed_out = TV_position_vector[pos,3]/2
		
		print "----------------->LOST: ",count
		if debug:
		#if theta_SV_orig_salvo == 0.0:
			print "TV_position_vector: ", TV_position_vector[pos_lost,1], TV_position_vector[pos_lost,0] , TV_position_vector[pos_lost,2], TV_position_vector[pos_lost,5]
			print "pos-lost: ", pos_lost
			print "SV:", longitude_SV, latitude_SV, heading_SV, speed_SV
			print "Heading_SV_adjusted: ",head_SV_adjusted
			print "thetas: ", theta_SV_orig_salvo, diff_teste, theta_teste
			print "Diff_theta: ", diff_teste
			print "D-Theta: ", d_theta_salvo
			print "Dist_lost_min: ", dist_lost_min
			print "Dist_lost: ", ((TV_position_vector[pos_lost,1] - longitude_SV)**2 + (TV_position_vector[pos_lost,0] - latitude_SV)**2)**0.5
			#val = math.atan2( TV_position_vector[pos_lost,1], TV_position_vector[pos_lost,0])
			#print "val: ", val
			#print "time: ", TV_position_vector[pos_lost,4]
			#print "current_time: ", tp.time()
			#print "Diff time: ", d_time
			print "shape: ", TV_position_vector.shape[0]
			for k in range(TV_position_vector.shape[0]):
				print TV_position_vector[k,0], TV_position_vector[k,1], TV_position_vector[k,2] , TV_position_vector[k,3] , TV_position_vector[k,4],TV_position_vector[k,5]
		count = count + 1
	else:
		speed_out = TV_position_vector[pos,3]
	lat_out = TV_position_vector[pos,0]				 #update the value of TV_latitude to be compared in bearing
	long_out = TV_position_vector[pos,1]				#update the value of TV_longitude to be compared in bearing

	TV_position_vector = TV_position_vector[pos:,:]	 #clean the vector, excluding the old positions
	if incremental_controller:
		TV_error_vector = TV_error_vector[pos:,:]

	
	return lat_out, long_out, heading_out, speed_out, TV_pid_out, TV_theta_out

#-----------------------------------------------------------------------
#FUNCAO: vehicle_general_control()
#	
#-----------------------------------------------------------------------
def vehicle_general_control():
	#TV data
	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV											#chamada para a variavel global
	global steering_TV									   #steering (angulacao das rodas do veiculo em relacao ao veiculo)
	global throttle_TV									   #percentual do acelerador (0 - 1)
	global brake_TV										  #percentual do freio (0 - 1)
	global speed_TV_reference
	global speed_TV_OLD
	global message_id_TV
	global message_id_TV_old
	#SV data
	global longitude_SV
	global latitude_SV
	global heading_SV
	global speed_SV											#chamada para a variavel global
	global steering_SV									   #steering (angulacao das rodas do veiculo em relacao ao veiculo), speed_TV
	global throttle_SV									   #percentual do acelerador (0 - 1)
	global brake_SV										  #percentual do freio (0 - 1)

	global theta_error

	global publication_error_topic
	global publication_topic
	global erro_vel
	global TV_position_vector							   #vector to store TV positions to be followed
	global mode

	global incremental_controller
	global TV_error_vector
	global TV_PID_error
	global TV_THETA_error

	debug = OFF

	pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
	msg = Control()																	#Define msg como um dado do tipo Control -> comando de controle do veiculo
	msg.header.stamp = rospy.Time.now();											#tempo atualizado da leitura
	msg.header.frame_id = "base_link";						
	msg.steer = 0 * ANG_TO_RAD														#angulacao desejada para as rodas (0-20) GRAUS

	#print "------------------------------------------------------"
	#calcula distancia
	distance = ((longitude_TV - longitude_SV)**2 + (latitude_TV - latitude_SV)**2)**0.5								 #real distance between TV and SV
	if abs(speed_TV - speed_TV_reference) > 1:
		speed_TV_reference = speed_TV
	maximum_distance = MINIMUM_DISTANCE + speed_TV_reference * SAFETY_TIME											  #desisred distance
	distance_error = distance - maximum_distance																		#error between desired ditance and real distance
	pid_error = distance_error

	#print "Distance", distance

	if distance_error < 0:
		#print "TV 1: ", longitude_TV, latitude_TV, heading_TV, speed_TV
		#print "SV: ", longitude_SV, latitude_SV, heading_SV, speed_SV
		if debug:
			print "Distance", distance
			print "maximum_distance: ", maximum_distance
			print "Distance Error: ", distance_error
	if (TV_position_vector[0,0]<=0.1 and TV_position_vector[0,1]<=0.1):												 #cleaning the first position of the vector
		TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV, tp.time(),message_id_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
		TV_position_vector = TV_position_vector[1:,:]																   #delete the first line with empty values
		if (incremental_controller):
			TV_error_vector = np.append(TV_error_vector,[[TV_PID_error, TV_THETA_error]],axis=0)
			TV_error_vector = TV_error_vector[1:,:]
	elif (message_id_TV_OLD <> message_id_TV):
		TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV,tp.time(), message_id_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
		if (incremental_controller):
			TV_error_vector = np.append(TV_error_vector,[[TV_PID_error, TV_THETA_error]],axis=0)
		
	#if debug:
		#print "TV_position_vector shape: ", TV_position_vector.shape
		#print "Speed_TV: ", speed_TV
		#print TV_position_vector
	safety_dist = ((speed_SV*3.6)*(speed_SV*3.6))/(250*0.75)		   #determine the safety distance considering a Mi = 1.0(coeficiente de atrito)
	if debug:
		print "Safety Distance: ", safety_dist
	#print "-----------------"
	#print "Speed_old: ", speed_TV_OLD
	#print "Speed_TV: ", speed_TV

	#ajusta velocidade e aceleracao
	#print "brake_TV", brake_TV
	if ((throttle_TV == -100.0 or brake_TV == 1.0) and FULL_BRAKE ) :
	#if (speed_TV_OLD - speed_TV) >= 0.08:  #TODO - send the BRAKE and ACCELERATION message as RAW messages
		msg.throttle, msg.brake = 0.0 , 1.0						#determine throttle and brake
		msg.steer = steering_SV
		if debug:
			print "-----------------"
			print "TV Brake "
			print "Longitude, Latitude: ", longitude_SV, latitude_TV
			print "Speed SV: ", speed_SV
			#TO DO - ADD a TIMER!!!!!
	#elif ((distance < safety_dist and speed_SV > 0.5) or distance < MINIMUM_DISTANCE):
	elif (distance < MINIMUM_DISTANCE):
		if (FULL_BRAKE):
			msg.throttle, msg.brake = 0.0 , 0.5						#determine throttle and brake
		else:
			msg.throttle, msg.brake = 0.0 , 1.0						#determine throttle and brake
		#lat_compare, long_compare, heading_compare, speed_compare, TV_dist_acum, TV_head_acum = compare_lost_position()		 #compares the SV_position with the vector of positions
		#msg.steer = direction_control (lat_compare, long_compare, heading_compare + TV_head_acum, heading_SV)
		msg.steer = steering_SV
		#msg.steer = 0.0
		#if debug:
		if debug:
			print "Distance < MINIMUM_distance"
			print "Distance Error: ", distance_error
			#print "maximun distance: ", maximum_distance
			print "Steering: ", steering_SV
	elif (message_id_TV_OLD == message_id_TV):											#if there are no new messages in buffer
		msg.throttle, msg.brake = throttle_SV,brake_SV			#determine throttle and brake
		msg.steer = steering_SV
	elif (speed_TV >= EDGE_LOW_SPEED and distance >= MINIMUM_DISTANCE):			   #TV is moving and the distance is significative
		if (distance < safety_dist and speed_SV > 0.5):
			distance_error = distance - safety_dist
		if incremental_controller:
			PID_dist = distance_control (distance_error + TV_PID_error)							 #determines de control action for the distance
		else:
			#print "distance error: ", distance_error
			PID_dist = distance_control (distance_error)										#determines de control action for the distance
		lat_compare, long_compare, heading_compare, speed_compare, TV_dist_acum, TV_head_acum = compare_lost_position()		 #compares the SV_position with the vector of positions
		new_speed_SV = speed_compare + PID_dist													 #defines the new speed for the SV
		if debug:
			print "TV Moving!"
			print "PID_dist: ", PID_dist
			print "speed_compare: ", speed_compare
			print "new Speed_SV: ", new_speed_SV
		msg.throttle, msg.brake = calc_throttle_brake(new_speed_SV,speed_SV)						#determine throttle and brake
		msg.steer = direction_control (lat_compare, long_compare, heading_compare + TV_head_acum, heading_SV)

		# if (new_speed_SV > speed_TV * 1.5) and distance < maximum_distance:
		#	 new_speed_SV = max(50.0/3.6,speed_TV)
		#	 print "REDUCED SPEED"
		#	 print "new Speed_SV: ", new_speed_SV
		#	 msg.throttle, msg.brake = calc_throttle_brake(new_speed_SV,speed_SV)						#determine throttle and brake

	else:
		msg.throttle, msg.brake = 0.0 , 1.0						#determine throttle and brake
		msg.steer = steering_SV
		if debug:
			if speed_TV < EDGE_LOW_SPEED:
				print "Speed < EDGE_LOW_SPEED"
			elif distance <= MINIMUM_DISTANCE:
				print "Distance < MINIMUM_distance"
			else:
				print "ERROR!!!" 

	# print "throttle: ", msg.throttle
	# print "brake: ", msg.brake
	# print "steer: ", msg.steer
	if debug:
		print "------------------------------"
	#Atualiza topico carro
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'
	
	#publishing error messages
	pub_error = rospy.Publisher(publication_error_topic, error_control, queue_size=1) 	#parametros de erro para controle e impressao
	msg_error = error_control() 
	msg_error.pid_error_value = distance_error
	msg_error.theta_error_value = theta_error
	msg_error.dist_tv = distance
	pub_error.publish(msg_error)


def control_data(data):
	global TV_PID_error
	global TV_THETA_error

	TV_PID_error = data.pid_error_value
	TV_THETA_error = data.theta_error_value

def SV_info(data):

	global longitude_SV
	global latitude_SV
	global heading_SV
	global speed_SV											#chamada para a variavel global
	global steering_SV									   #steering (angulacao das rodas do veiculo em relacao ao veiculo)
	global throttle_SV									   #percentual do acelerador (0 - 1)
	global brake_SV										  #percentual do freio (0 - 1)
	global message_id_SV

	#print 'SV_INFO'

	latitude_SV = data.latitude
	longitude_SV = data.longitude
	heading_SV = data.heading_headingValue
	speed_SV = data.speed_speedValue						# salva o valor da velocidade no instante
	steering_SV = data.steeringWheelAngle_steeringWheelAngleValue
	throttle_SV = data.gasPedalPercent_Value
	brake_SV = data.brakePedalPercent_Value
	message_id_SV = data.yawRate_Value

	# global count_lap
	# global lap
	# if (abs(longitude_TV-180) < 1 and abs(latitude_TV-10) < 2):  # conta o numero de voltas
	#	 if count_lap == 0:
	#		 lap = lap +1
	#		 count_lap = count_lap + 1
	#		 print "Lap: ", lap
	#		 print "TV_data: ", latitude_TV, longitude_TV, heading_TV, speed_TV
	#		 print "SV_data: ", latitude_SV, longitude_SV, heading_SV, speed_SV
	# else:
	#	 count_lap = 0

	vehicle_general_control()

def TV_info(data):

	global car_name_TV
	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV											 #chamada para a variavel global
	global steering_TV									   #steering (angulacao das rodas do veiculo em relacao ao veiculo)
	global throttle_TV									   #percentual do acelerador (0 - 1)
	global brake_TV										  #percentual do freio (0 - 1)
	global message_id_TV
	global message_id_TV_OLD

	global speed_TV_OLD
	global heading_TV_OLD

	global lap
	global count_lap

	global TV_position_vector							   #vector to store TV positions to be followed

	global gen_cam_id									#used to generate CAM_ID for eachmessage. Use with RXNetwork, remove with RXNetwork_OMNET
	#print 'TV_INFO'

	debug = OFF

	if (car_name_TV == data.car_name):
		if debug:
			print "-----------------"
			#print "Speed_old: ", speed_TV_OLD
			#print "Speed_TV: ", speed_TV
			#print "Data.Speed: ", data.speed_speedValue
			print "Data.ID: ", data.yawRate_Value
			print "message_id_TV: ", message_id_TV

		if (speed_TV_OLD <> data.speed_speedValue): 
			if debug:
				print "speed_TV_OLD <> data.speed_speedValue"
			speed_TV_OLD = speed_TV									#save the old value of SPEED TV for desaccelaration calculus
		
		if (message_id_TV_OLD <> message_id_TV): 
			if debug:
				print "message_id_TV_OLD <> data.message_id_TV"
			message_id_TV_OLD = message_id_TV									#save the old value of SPEED TV for desaccelaration calculus


		latitude_TV = data.latitude
		longitude_TV = data.longitude
		heading_TV_OLD = heading_TV									#save the old heading_TV for bearing calculos
		heading_TV = data.heading_headingValue
		speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
		steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
		throttle_TV = data.gasPedalPercent_Value
		brake_TV = data.brakePedalPercent_Value
		if (not gen_cam_id):
			message_id_TV = data.yawRate_Value
		else: 
			message_id_TV = message_id_TV + 1
		#print "TV 0: ", longitude_TV, latitude_TV, heading_TV, speed_TV, car_name_TV

	


def listener():
	global TV_topic
	global SV_topic
	global car_name_TV
	global TV_control_topic
	global incremental_controller
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	print "TV Topic: ", TV_topic
	print "SV Topic: ", SV_topic
	rospy.init_node('listener', anonymous=True)

	if incremental_controller:
		print "TV Control topic: ", TV_control_topic
		rospy.Subscriber(TV_control_topic,error_control,control_data)

	rospy.Subscriber(TV_topic,CAM_simplified,TV_info)
	rospy.Subscriber(SV_topic,CAM_simplified,SV_info)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	#global gen_cam_id
	param = sys.argv[1:]								#le os dados passados como argumentos (TV SV)
														#param[0] - TV
														#param[1] - SV
														#param[2] - CONTROL_MODE
	car_name_TV = "/" + param[0] +"/"
	#TODO - tratamento de erros para parametros de entrada
	#TODO - Tornar generico parametros de leitura
	#TV_topic = "/" + param[0] +"/carINFO"				   #cria a string do topico do TV a ser lido (INFO Padrao)
	#TV_topic = "/" + param[1] +"/RXNetwork"				  #string to be readed using a simulation of communication
	#gen_cam_id = ON											#used to generate CAM_ID for each message. Use with RXNetwork, remove with RXNetwork_OMNET						
	TV_topic = "/" + param[1] +"/RXNetwork_OMNET"			  #string to be used with OMNET
	gen_cam_id = OFF											#used to generate CAM_ID for each message. Use with RXNetwork, remove with RXNetwork_OMNET						
	SV_topic = "/" + param[1] +"/carINFO"				   #cria a string do topico do SV a ser lido
	publication_topic = "/" + param[1] +"/prius"			#cria a string do topico do SV a ser publicado (controle)
	publication_error_topic = "/" + param[1] + '/error_control'

	if 'control' in param:								  #enable the incremental controller of PID
		TV_control_topic = "/" + param[0] + '/error_control'
		incremental_controller = ON

	#pub_2 = rospy.Publisher(param[1]+'/error_control', error_control, queue_size=1) 	#parametros de erro para controle e impressao
	listener()