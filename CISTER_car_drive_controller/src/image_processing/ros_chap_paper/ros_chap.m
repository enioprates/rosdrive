%NETWORK PERFORMANCE
%%
%% ajustando formato de dados
%format long;
%format compact;
format longG;
%close all;
%clear;
%clc;
%% Loading Data

%RUN THE SCRIPT 'NET_TRACK_DATA.M'
ros_chap_data

%% COLUMNS IDs
% [rospy.get_time()-time_base, 
% count_TV,
%latitude_TV, 
%longitude_TV,
%speed_TV, 
%speed_TV * 3.6, 
%heading_TV,
%throttle_TV,
%brake_TV, 
%lap_TV]
%1 - time;
%2 - count_SV;
%3 - latitude_SV;
%4 - longitude_SV;
%5 - speed_SV;
%6 - speed_TV;
%7 - Speed_error
%8 - heading_SV;
%9 - PID Error;
%10 - Theta Error;
%11 - lap_SV;
%12 - distance;

tv_time         = 1;
tv_count        = 2;
tv_lat          = 3;
tv_long         = 4;
tv_speed        = 5;
tv_speed_erro   = 6;
tv_head         = 7;
tv_head_err_graus = 8;
tv_head_err_rad = 9;
tv_throttle     = 10;
tv_brake        = 11;
tv_lap          = 12;
s1              = 13;
s2              = 14;
s3              = 15;
s4              = 16;
s5              = 17;
s6              = 18;


%% trajectory 1st Curve
%-------------------------------------------------------------------------
%Comparando a trajetória dos veiculos na pista
%Graficos obtidos diretamente dos dados de Latitude e Longitude de cada
%veiculo
%-------------------------------------------------------------------------
figure
plot(data_sv_t_13_10_00_00(:,tv_lat)',data_sv_t_13_10_00_00(:,tv_long)','-')
hold on
plot(data_sv_t_15_10_00_010(:,tv_lat)',data_sv_t_15_10_00_010(:,tv_long)','-')
plot(data_sv_t_15_10_00_005(:,tv_lat)',data_sv_t_15_10_00_005(:,tv_long)','-')
plot(data_sv_t_15_10_00_00(:,tv_lat)',data_sv_t_15_10_00_00(:,tv_long)','-')
plot(data_sv_t_15_10_01_010(:,tv_lat)',data_sv_t_15_10_01_010(:,tv_long)','-')
plot(data_sv_t_15_10_005_00(:,tv_lat)',data_sv_t_15_10_005_00(:,tv_long)','-')
plot(data_sv_t_15_10_005_005(:,tv_lat)',data_sv_t_15_10_005_005(:,tv_long)','-')
plot(data_sv_t_15_10_010_00(:,tv_lat)',data_sv_t_15_10_010_00(:,tv_long)','-')

title('Trajectory Comparison - Curve 7');
xlabel ('Longitude (m)');
ylabel('Latitude (m)');
%yticks(3:0.1:4)
legend('REF','PID5 - 10-00-00','PID6 - 10-00-0.5','PID7 - 10-00-01', 'PID8 - 10-01-01', 'PID9 - 10-0.5-00', 'PID10 - 10-0.5-0.5', 'PID11 - 10-01-00');
hold off
%% Theta Error
figure
plot(data_sv_t_13_10_00_00(:,tv_lat)',data_sv_t_13_10_00_00(:,tv_head_err_graus)','-')
hold on
plot(data_sv_t_15_10_00_010(:,tv_lat)',data_sv_t_15_10_00_010(:,tv_head_err_graus)','--')
plot(data_sv_t_15_10_00_005(:,tv_lat)',data_sv_t_15_10_00_005(:,tv_head_err_graus)','-')
plot(data_sv_t_15_10_00_00(:,tv_lat)',data_sv_t_15_10_00_00(:,tv_head_err_graus)','-')
plot(data_sv_t_15_10_01_010(:,tv_lat)',data_sv_t_15_10_01_010(:,tv_head_err_graus)','-.')
plot(data_sv_t_15_10_005_00(:,tv_lat)',data_sv_t_15_10_005_00(:,tv_head_err_graus)',':')
plot(data_sv_t_15_10_005_005(:,tv_lat)',data_sv_t_15_10_005_005(:,tv_head_err_graus)','-.')
plot(data_sv_t_15_10_010_00(:,tv_lat)',data_sv_t_15_10_010_00(:,tv_head_err_graus)',':')

title('Heading Error (Degrees)');
xlabel ('Time (s)');
ylabel('Error (Degree)');
%yticks(3:0.1:4)
legend('REF','PID5 - 10-00-00','PID6 - 10-00-0.5','PID7 - 10-00-01', 'PID8 - 10-01-01', 'PID9 - 10-0.5-00', 'PID10 - 10-0.5-0.5', 'PID11 - 10-01-00');
hold off

%% trajectory - TFULL
%-------------------------------------------------------------------------
%Comparando a trajetória dos veiculos na pista
%Graficos obtidos diretamente dos dados de Latitude e Longitude de cada
%veiculo
%-------------------------------------------------------------------------
figure
plot(data_sv_tf_13_10_000_000(:,tv_lat)',data_sv_tf_13_10_000_000(:,tv_long)','-')
hold on
plot(data_sv_tf_15_10_000_010(:,tv_lat)',data_sv_tf_15_10_000_010(:,tv_long)','-')
plot(data_sv_tf_15_10_000_005(:,tv_lat)',data_sv_tf_15_10_000_005(:,tv_long)','--')
plot(data_sv_tf_15_10_000_000(:,tv_lat)',data_sv_tf_15_10_000_000(:,tv_long)','-.')

title('Trajectory Comparison - Full Circuit');
xlabel ('Longitude (m)');
ylabel('Latitude (m)');
%yticks(3:0.1:4)
legend('REF','PID5 - 10-00-00','PID6 - 10-00-0.5','PID7 - 10-00-01');
hold off

%% Box Plot - PID ERROR
%-------------------------------------------------------------------------
%Comparando o erro de DISTANCIA de cada carro em um BOX PLOT
%Ajuste de posição inicial para excluir o erro de posição inicial
%Todos os dados do BOX PLOT devem ter o mesmo tamanho. O min_size é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

used_coluumn = tv_head_err_graus;         %coluna a ser utilizada para a comparação

series = 4;
data_size = 1;
inicio = 1;                      %inicio: Posicao apos o erro inicial (estabiliza o sistema)
min_size = 10266;                    %min_size: tamanho do menor conjunto de dados

k = zeros(min_size-inicio+1, series * data_size); i=1;  %matriz que armazena os dados do box plot
%k(:,i) = data_tv_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_tf_13_10_000_000(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_tf_15_10_000_010(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_tf_15_10_000_005(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_tf_15_10_000_000(inicio:min_size,used_coluumn); i = i+1;

labels = {'REF','PID5','PID6','PID7'};

figure
boxplot(k,'Whisker',10, 'Labels', labels)
hold on
title('Heading Error - Full Circuit');
xlabel ('Heading Controller (HC)');
ylabel('Error (Degrees)');
xtickangle(45)
hold off

%% trajectory - TFULL SOA
%-------------------------------------------------------------------------
%Comparando a trajetória dos veiculos na pista
%Graficos obtidos diretamente dos dados de Latitude e Longitude de cada
%veiculo
%-------------------------------------------------------------------------
figure
plot(curve_oa_13_10_00_00(:,tv_lat)',curve_oa_13_10_00_00(:,tv_long)','-')
hold on
plot(curve_oa_15_10_00_10(:,tv_lat)',curve_oa_15_10_00_10(:,tv_long)','-')
plot(curve_oa_15_10_00_05(:,tv_lat)',curve_oa_15_10_00_05(:,tv_long)','--')
plot(curve_oa_15_10_00_00(:,tv_lat)',curve_oa_15_10_00_00(:,tv_long)','-.')

title('Trajectory Comparison - DA Static Obstacles');
xlabel ('Longitude (m)');
ylabel('Latitude (m)');
%yticks(3:0.1:4)
legend('REF','PID5 - 10-00-00','PID6 - 10-00-0.5','PID7 - 10-00-01');
hold off

%% Box Plot - PID ERROR SOA
%-------------------------------------------------------------------------
%Comparando o erro de DISTANCIA de cada carro em um BOX PLOT
%Ajuste de posição inicial para excluir o erro de posição inicial
%Todos os dados do BOX PLOT devem ter o mesmo tamanho. O min_size é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

used_coluumn = tv_head_err_graus;         %coluna a ser utilizada para a comparação

series = 4;
data_size = 1;
inicio = 1;                      %inicio: Posicao apos o erro inicial (estabiliza o sistema)
min_size = 9593;                    %min_size: tamanho do menor conjunto de dados

k = zeros(min_size-inicio+1, series * data_size); i=1;  %matriz que armazena os dados do box plot
%k(:,i) = data_tv_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = curve_oa_13_10_00_00(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = curve_oa_15_10_00_10(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = curve_oa_15_10_00_05(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = curve_oa_15_10_00_00(inicio:min_size,used_coluumn); i = i+1;

labels = {'REF','PID5','PID6','PID7'};

figure
boxplot(k,'Whisker',10, 'Labels', labels)
hold on
title('Heading Error - DA Static Obstacles');
xlabel ('Heading Controller');
ylabel('Error (Degrees)');
xtickangle(45)
hold off

%% Obstacle Distance - SOA

soa = zeros(4,3);
i=1;
A = curve_oa_13_10_00_00(:,s1:s4);soa(i,1)=min(A(A>0));i=i+1;
A = curve_oa_15_10_00_10(:,s1:s4);soa(i,1)=min(A(A>0));i=i+1;
A = curve_oa_15_10_00_05(:,s1:s4);soa(i,1)=min(A(A>0));i=i+1;
A = curve_oa_15_10_00_00(:,s1:s4);soa(i,1)=min(A(A>0));i=1;

A = curve_oa_13_10_00_00(:,s1:s4);soa(i,2)=max(A(A>0));i=i+1;
A = curve_oa_15_10_00_10(:,s1:s4);soa(i,2)=max(A(A>0));i=i+1;
A = curve_oa_15_10_00_05(:,s1:s4);soa(i,2)=max(A(A>0));i=i+1;
A = curve_oa_15_10_00_00(:,s1:s4);soa(i,2)=max(A(A>0));i=1;

A = curve_oa_13_10_00_00(:,s1:s4);soa(i,2)=mean(A(A>0));i=i+1;
A = curve_oa_15_10_00_10(:,s1:s4);soa(i,2)=mean(A(A>0));i=i+1;
A = curve_oa_15_10_00_05(:,s1:s4);soa(i,2)=mean(A(A>0));i=i+1;
A = curve_oa_15_10_00_00(:,s1:s4);soa(i,2)=mean(A(A>0));i=1;

soa

%% trajectory - TFULL DOA

figure
plot(curve_2v_13_12_2(:,tv_time)',curve_2v_13_12_2(:,tv_lat)','--')
hold on
plot(curve_2v_13_12_1(:,tv_time)',curve_2v_13_12_1(:,tv_lat)','-')
plot(curve_2v_14_12_1(:,tv_time)',curve_2v_14_12_1(:,tv_lat)','-')
plot(curve_2v_15_12_1(:,tv_time)',curve_2v_15_12_1(:,tv_lat)','-')
plot(curve_2v_16_12_1(:,tv_time)',curve_2v_16_12_1(:,tv_lat)','-')
plot(curve_2v_20_12_1(:,tv_time)',curve_2v_20_12_1(:,tv_lat)','-')

title('Trajectory Comparison - DA Dynamic Obstacles');
xlabel ('Time (s)');
ylabel('Longitude (m)');
%yticks(3:0.1:4)
legend('REF - 12 m/s','13 m/s','14 m/s','15 m/s','16 m/s','20 m/s');
hold off

%% trajectory - TFULL DOA

figure
plot(curve_2v_13_12_2(:,tv_lat)',curve_2v_13_12_2(:,tv_long)','--')
hold on
plot(curve_2v_13_12_1(:,tv_lat)',curve_2v_13_12_1(:,tv_long)','-')
plot(curve_2v_14_12_1(:,tv_lat)',curve_2v_14_12_1(:,tv_long)','-')
plot(curve_2v_15_12_1(:,tv_lat)',curve_2v_15_12_1(:,tv_long)','-')
plot(curve_2v_16_12_1(:,tv_lat)',curve_2v_16_12_1(:,tv_long)','-')
plot(curve_2v_20_12_1(:,tv_lat)',curve_2v_20_12_1(:,tv_long)','-')

title('Trajectory Comparison - DA Dynamic Obstacles');
xlabel ('Time (s)');
ylabel('Longitude (m)');
ylim([39 47])
legend('REF - 12 m/s','13 m/s','14 m/s','15 m/s','16 m/s','20 m/s');
hold off


%% Speed
%-------------------------------------------------------------------------
%Comparando a Velocidade dos veiculos na pista
%-------------------------------------------------------------------------
%plot(data_sv_s_01_01(:,tv_time)',data_sv_s_01_01(:,tv_speed)','-')

plot(data_sv_s_07_01(:,tv_time)',data_sv_s_07_01(:,tv_speed)',':')  %PID (10,0,0)
hold on
plot(data_sv_s_04_01(:,tv_time)',data_sv_s_04_01(:,tv_speed)','-.') %PID (10.8,2.16,0.270)
plot(data_sv_s_09_01(:,tv_time)',data_sv_s_09_01(:,tv_speed)','--') %PID (10.8,4.32,0.135)
plot(data_sv_s_06_01(:,tv_time)',data_sv_s_06_01(:,tv_speed)','-')  %PID (10.8,2.16,0.135)

%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_speed)','-')
%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_throttle)','-')

title('Speed Comparison');
xlabel ('Time (s)');
ylabel('Speed (m/s)');
%yticks(3:0.1:4)
legend('PID1 - 10.8-0.0-0.0','PID2 - 10.8-2.16-0.270','PID3 - 10.8-4.32-0.135','PID4 - 10.8-2.16-0.135');
hold off

%% Speed Error
%-------------------------------------------------------------------------
%Comparando a trajetória dos veiculos na pista
%Graficos obtidos diretamente dos dados de Latitude e Longitude de cada
%veiculo
%-------------------------------------------------------------------------
plot(data_sv_s_01_01(:,tv_time)',data_sv_s_01_01(:,tv_speed_erro)','-')
hold on
plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_speed_erro)','-')
plot(data_sv_s_03_01(:,tv_time)',data_sv_s_03_01(:,tv_speed_erro)','-')
plot(data_sv_s_04_01(:,tv_time)',data_sv_s_04_01(:,tv_speed_erro)','-')
plot(data_sv_s_05_01(:,tv_time)',data_sv_s_05_01(:,tv_speed_erro)','-')
plot(data_sv_s_06_01(:,tv_time)',data_sv_s_06_01(:,tv_speed_erro)','-')
plot(data_sv_s_07_01(:,tv_time)',data_sv_s_07_01(:,tv_speed_erro)','-')
plot(data_sv_s_08_01(:,tv_time)',data_sv_s_08_01(:,tv_speed_erro)','-')
plot(data_sv_s_09_01(:,tv_time)',data_sv_s_09_01(:,tv_speed_erro)','-')

%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_speed)','-')
%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_throttle)','-')

title('Speed Error Comparison (P)');
xlabel ('Time (s)');
ylabel('Speed (m/s)');
%yticks(3:0.1:4)
legend('005','02','02.5','03','03 1.0','05','10','10.8','10.8 - 2');
hold off

%% Accel
%-------------------------------------------------------------------------
%Comparando a trajetória dos veiculos na pista
%Graficos obtidos diretamente dos dados de Latitude e Longitude de cada
%veiculo
%-------------------------------------------------------------------------
plot(data_sv_s_01_01(:,tv_time)',data_sv_s_01_01(:,tv_throttle)','-')
hold on
plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_throttle)','-')
%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_speed)','-')
%plot(data_sv_s_02_01(:,tv_time)',data_sv_s_02_01(:,tv_throttle)','-')

title('Trajectory Comparison (P)');
xlabel ('Time (s)');
ylabel('Speed (m/s)');
%yticks(3:0.1:4)
legend('02','03');
hold off

%% Box Plot - PID ERROR
%-------------------------------------------------------------------------
%Comparando o erro de DISTANCIA de cada carro em um BOX PLOT
%Ajuste de posição inicial para excluir o erro de posição inicial
%Todos os dados do BOX PLOT devem ter o mesmo tamanho. O min_size é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

used_coluumn = sv_dist_err;         %coluna a ser utilizada para a comparação

series = 7;
data_size = 1;
inicio = 1250;                      %inicio: Posicao apos o erro inicial (estabiliza o sistema)
min_size = 6180;                    %min_size: tamanho do menor conjunto de dados

k = zeros(min_size-inicio+1, series * data_size); i=1;  %matriz que armazena os dados do box plot
%k(:,i) = data_tv_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_1_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_2_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_3_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_4_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_5_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_6_06(inicio:min_size,used_coluumn); i = i+1;
k(:,i) = data_sv_7_06(inicio:min_size,used_coluumn);

labels = {'BSP', 'BSP-P', 'CSP1','CSP2', 'CSP3', 'CSP4', 'CSP5'};

figure
boxplot(k,'Whisker',10, 'Labels', labels)
hold on
title('SC3 - SV5 Distance Error');
xlabel ('Communication Profile');
ylabel('Error (m)');
xtickangle(45)
hold off


%% LINE PLOT - DISTANCE ERROR
%-------------------------------------------------------------------------
%Comparando o erro de DISTANCIA de cada carro em um LINE PLOT
%Ajuste de posição inicial para excluir o erro de posição inicial
%Dados ajustados para o mesmo tamanho. O final é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

coluna_1 = sv_count;
coluna_2 = sv_dist_err;

initial = 1250; final = 6180;
%initial = 1250; final = 6608;

plot(data_sv_1_05(initial:final,coluna_1)',data_sv_1_05(initial:final,coluna_2)',':')
hold on
plot(data_sv_2_05(initial:final,coluna_1)',data_sv_2_05(initial:final,coluna_2)',':')
plot(data_sv_3_05(initial:final,coluna_1)',data_sv_3_05(initial:final,coluna_2)','-')
plot(data_sv_4_05(initial:final,coluna_1)',data_sv_4_05(initial:final,coluna_2)','-')
plot(data_sv_5_05(initial:final,coluna_1)',data_sv_5_05(initial:final,coluna_2)','-.')
plot(data_sv_6_05(initial:final,coluna_1)',data_sv_6_05(initial:final,coluna_2)','-.')
plot(data_sv_7_05(initial:final,coluna_1)',data_sv_7_05(initial:final,coluna_2)','-.')

hold off

title('SV5 Distance Error');
xlabel ('Time (s)');
ylabel('Distance (m)');
%yticks(3:0.1:4)
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4', 'CSP5');
hold off

%% ECDF - ERRO DE DISTANCIA
%-------------------------------------------------------------------------
%EMPIRICAL CUMULATIVE DISTRIBUTION FUNCTION
%Ajuste de posição inicial para excluir o erro de posição inicial
%Dados ajustados para o mesmo tamanho. O final é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

coluna_used = sv_dist_err;
initial = 1200; final = 6180;

[F,X] = ecdf(data_sv_1_06(initial:final,coluna_used)); stairs(X,F,'--');
hold on
[F,X] = ecdf(data_sv_2_06(initial:final,coluna_used)); stairs(X,F,'--');
[F,X] = ecdf(data_sv_3_06(initial:final,coluna_used)); stairs(X,F,'-');
[F,X] = ecdf(data_sv_4_06(initial:final,coluna_used)); stairs(X,F,'-');
[F,X] = ecdf(data_sv_5_06(initial:final,coluna_used)); stairs(X,F,'-.');
[F,X] = ecdf(data_sv_6_06(initial:final,coluna_used)); stairs(X,F,'-.');
[F,X] = ecdf(data_sv_7_06(initial:final,coluna_used)); stairs(X,F,'-.');
hold off
title('SC3 - SV5 ECDF of Distance Error');
xlabel ('Error (m)');
ylabel('ECDF');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4' , 'CSP5');



%% CDF - NORMAL

coluna_used = sv_dist_err;
initial = 1200; final = 6180;

figure
x = data_sv_1_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,':')
hold on
x = data_sv_2_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,':')
x = data_sv_3_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,'-')
x = data_sv_4_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,'-')
x = data_sv_5_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,'-.')
x = data_sv_2_06(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,'-.')
x = data_sv_2_07(initial:final,coluna_used);            phat = mle(x);
pd2 = makedist('Normal','mu',phat(1),'sigma',phat(2));  p = cdf(pd2,x);
plot(x,p,'-.')

title('SV5 Distance Error CDF');
xlabel ('Distance Error (m)');
ylabel('CDF');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');

hold off


%% PERCENTIL

percentil = 95;
coluna_used = sv_dist_err;
initial = 1250; final = 6180;

series = 7;
data_size = 5;

data_percentil = zeros(series, data_size); i=1;j=1;


%x = data_sv_1_02(initial:final,coluna_used); 
data_percentil(i,j) = prctile(data_sv_1_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_1_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_1_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_1_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_1_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_2_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_2_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_2_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_2_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_2_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_3_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_3_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_3_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_3_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_3_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_4_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_4_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_4_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_4_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_4_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_5_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_5_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_5_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_5_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_5_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_6_02(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_6_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_6_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_6_05(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_6_06(initial:final,coluna_used),percentil); j=1; i=i+1;

data_percentil(i,j) = prctile(data_sv_7_02(initial:final,coluna_used)/2.2,percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_7_03(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_7_04(initial:final,coluna_used),percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_7_05(initial:final,coluna_used)/1.5,percentil); j=j+1;
data_percentil(i,j) = prctile(data_sv_7_06(initial:final,coluna_used),percentil); j=1; i=i+1;

figure
%labels = {'SV1','SV2','SV3','SV4','SV5'};
labels = {'BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4', 'CSP5'};

bar(data_percentil)

% i=1;
% plot(data_percentil(i,:),':'); i=i+1;
% hold on
% plot(data_percentil(i,:),':'); i=i+1;
% plot(data_percentil(i,:),'-'); i=i+1;
% plot(data_percentil(i,:),'-'); i=i+1;
% plot(data_percentil(i,:),'-.'); i=i+1;
% plot(data_percentil(i,:),'-.');

xticks(1:7);
xticklabels(labels);

title('SC3 - 95% Distance Error');
xlabel ('Follower');
ylabel('95% Error (m)');
%legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');
legend('SV1','SV2','SV3','SV4','SV5');

hold off

%% Stability
used_coluumn = sv_dist_err;
%Zerando a coluna do tempo
norm_vec = zeros(5,1);

initial = 1200;
final = 6180;

series = 6;
data_size = 5;

data_stabil = zeros(series, data_size); i=1;j=1;

data_stabil(i,j) = norm(data_sv_1_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_1_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_1_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_1_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_1_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_2_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_2_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_2_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_2_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_2_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_3_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_3_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_3_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_3_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_3_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_4_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_4_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_4_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_4_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_4_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_5_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_5_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_5_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_5_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_5_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_6_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_6_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_6_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_6_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_6_06(initial:final,used_coluumn),inf); j=1; i=i+1;

data_stabil(i,j) = norm(data_sv_7_02(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_7_03(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_7_04(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_7_05(initial:final,used_coluumn),inf); j=j+1;
data_stabil(i,j) = norm(data_sv_7_06(initial:final,used_coluumn),inf); j=1; i=i+1;

figure
%labels = {'SV1','SV2','SV3','SV4','SV5'};
labels = {'BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4', 'CSP5'};

bar(data_stabil)

% i=1;
% plot(data_stabil(i,:),':'); i=i+1;
% hold on
% plot(data_stabil(i,:),':'); i=i+1;
% plot(data_stabil(i,:),'-'); i=i+1;
% plot(data_stabil(i,:),'-'); i=i+1;
% plot(data_stabil(i,:),'-.'); i=i+1;
% plot(data_stabil(i,:),'-.');

xticks(1:7);
xticklabels(labels);

title('Co-VP Stability Check');
xlabel ('Follower');
ylabel('Error (m)');
%legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');
legend('SV1','SV2','SV3','SV4','SV5');

hold off


%% Heading Comparison General (Compara toda a PISTA)

used_column = sv_head;
time_column = sv_time;


plot(data_tv(:,time_column),data_tv(:,7))
hold on
plot(data_sv_1_06(:,time_column),data_sv_1_06(:,used_column),':')
plot(data_sv_2_06(:,time_column),data_sv_2_06(:,used_column),':')
plot(data_sv_3_06(:,time_column),data_sv_3_06(:,used_column),'-')
plot(data_sv_4_06(:,time_column),data_sv_4_06(:,used_column),'-')
plot(data_sv_5_06(:,time_column),data_sv_5_06(:,used_column),'-.')
plot(data_sv_6_06(:,time_column),data_sv_6_06(:,used_column),'-.')
plot(data_sv_7_06(:,time_column),data_sv_7_06(:,used_column),'-.')
hold off
title('SV5 General Heading Comparison');
xlabel ('Time (s)');
ylabel('Heading (rad)');
legend('Leader','BSP','BSP-P','CSP1','CSP2','CSP3','CSP4','CSP5');



%% Heading Comparison Spec (Compara apenas as curvas 2 e 3)

used_column = sv_head;
time_column = sv_time;

initial = 3000; final = 5220;
figure
plot(data_tv(initial:final,sv_time),data_tv(initial:final,7))
hold on
initial = 3420; final = 5650;
plot(data_sv_1_06(initial:5550,sv_time),data_sv_1_06(initial:5550,used_column),':')
plot(data_sv_2_06(initial:5550,sv_time),data_sv_2_06(initial:5550,used_column),':')
plot(data_sv_3_06(initial:final,sv_time),data_sv_3_06(initial:final,used_column),'-')
plot(data_sv_4_06(initial:final,sv_time),data_sv_4_06(initial:final,used_column),'-')
plot(data_sv_5_06(initial:5630,sv_time),data_sv_5_06(initial:5630,used_column),'-.')
plot(data_sv_6_06(initial:final,sv_time),data_sv_6_06(initial:final,used_column),'-.')

hold off
title('SV5 Heading Comparison - Curves 2 and 3');
xlabel ('Time (s)');
ylabel('Heading (rad)');
legend('Leader','BSP','BSP-P','CSP1','CSP2','CSP3','CSP4');

%% Heading error comparison - Line PLOT  (CURVAS 2 e 3)

coluna_1 = sv_time;
coluna_2 = sv_theta_err;

initial = 3420; final = 5650;
figure
plot(data_sv_1_06(initial:5550,coluna_1)',data_sv_1_06(initial:5550,coluna_2)',':')
hold on
plot(data_sv_2_06(initial:5550,coluna_1)',data_sv_2_06(initial:5550,coluna_2)',':')
plot(data_sv_3_06(initial:final,coluna_1)',data_sv_3_06(initial:final,coluna_2)','-')
plot(data_sv_4_06(initial:final,coluna_1)',data_sv_4_06(initial:final,coluna_2)','-')
plot(data_sv_5_06(initial:5630,coluna_1)',data_sv_5_06(initial:5630,coluna_2)','-.')
plot(data_sv_6_06(initial:final,coluna_1)',data_sv_6_06(initial:final,coluna_2)','-.')

hold off

title('SV5 Distance Error');
xlabel ('Time (s)');
ylabel('Distance (m)');
%yticks(3:0.1:4)
%legend('TV');
% legend('TV','SV2');
% legend('TV','SV2','SV3');
% legend('TV','SV2','SV3','SV4');
%legend('TV','SV2','SV3','SV4','SV5');
% legend('TV','SV2','SV3','SV4','SV5','SV6');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8');
%legend('003','005','007','010','030','050','BSP1','BSPP1','CSP1','BSP2','BSPP2','CSP2');
%legend('003','005','007','010','BSP1','BSPP1','CSP1','BSP2','BSPP2','CSP2');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');
hold off


%% SV5-SV4 Heading Error Comparison - line plot

used_column = sv_head;
time_column = sv_time;

figure
size_data = 6521; series = 6;
diff = zeros(size_data,series);
lim = 3.14;

data1 =  data_sv_1_05; 
data2 =  data_sv_1_06; j=1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end
data1 =  data_sv_2_05; 
data2 =  data_sv_2_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_3_05; 
data2 =  data_sv_3_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_4_05; 
data2 =  data_sv_4_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_5_05; 
data2 =  data_sv_5_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_6_05; 
data2 =  data_sv_6_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end
j=1;
plot(data_sv_1_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
hold on
plot(data_sv_2_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
plot(data_sv_3_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_4_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_5_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
plot(data_sv_6_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
hold off
title('SV5 Heading Error Comparison');
xlabel ('Time (s)');
ylabel('Error (rad)');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');

%% SV5-SV4 Heading Error Comparison - box plot

used_column = sv_head;
time_column = sv_time;

size_data = 6521; series = 6;
diff = zeros(size_data,series);
lim = 3.14;

data1 =  data_sv_1_05; 
data2 =  data_sv_1_06; j=1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end
data1 =  data_sv_2_05; 
data2 =  data_sv_2_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_3_05; 
data2 =  data_sv_3_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_4_05; 
data2 =  data_sv_4_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_5_05; 
data2 =  data_sv_5_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

data1 =  data_sv_6_05; 
data2 =  data_sv_6_06;j=j+1;
for i=1:size_data
    if (abs(data1(i,used_column) - data2(i,used_column))<3.14)
        diff(i,j)= data1(i,used_column) - data2(i,used_column);
    else
        diff(i,j)= data1(i,used_column) + data2(i,used_column);
    end
end

labels = {'BSP', 'BSP-P', 'CSP1','CSP2', 'CSP3', 'CSP4'};

figure
boxplot(diff,'Whisker',20, 'Labels', labels)
hold on
title('SV5 Heading Error');
%xlabel ('Vehicle (s)');
ylabel('Error (rad)');
xtickangle(45)
hold off

%% SV5 SV1 Heading error comparison adjust 
% (Encontrando o defasamento de tempo entre SV1 e SV5 em cada cenário)

coluna_1 = sv_count;
coluna_2 = sv_head;

%initial = 3420; final = 5650;
initial = 1;final = 6521;
figure
plot(data_sv_6_01(initial:final,coluna_1)'+406,data_sv_6_01(initial:final,7)',':')
hold on
plot(data_sv_6_06(initial:final,coluna_1)',data_sv_6_06(initial:final,coluna_2)',':')
% plot(data_sv_3_06(initial:final,coluna_1)',data_sv_3_06(initial:final,coluna_2)','-')
% plot(data_sv_4_06(initial:final,coluna_1)',data_sv_4_06(initial:final,coluna_2)','-')
% plot(data_sv_5_06(initial:5630,coluna_1)',data_sv_5_06(initial:5630,coluna_2)','-.')
% plot(data_sv_6_06(initial:final,coluna_1)',data_sv_6_06(initial:final,coluna_2)','-.')

hold off

title('SV5 Distance Error');
xlabel ('Time (s)');
ylabel('Distance (m)');
%yticks(3:0.1:4)
%legend('TV');
% legend('TV','SV2');
% legend('TV','SV2','SV3');
% legend('TV','SV2','SV3','SV4');
%legend('TV','SV2','SV3','SV4','SV5');
% legend('TV','SV2','SV3','SV4','SV5','SV6');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8');
%legend('003','005','007','010','030','050','BSP1','BSPP1','CSP1','BSP2','BSPP2','CSP2');
%legend('003','005','007','010','BSP1','BSPP1','CSP1','BSP2','BSPP2','CSP2');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');
hold off

sv1_desl = 380;
sv2_desl = 375;
sv3_desl = 395;
sv4_desl = 412;
sv5_desl = 385;
sv6_desl = 406;
sv7_desl = 406;



%% SV5-SV1 Heading Error Comparison - line plot

used_column1 = 7;
used_column2 = sv_head;

size_data = 6521; series = 7;
initial = 1; final = 6521;
diff = zeros(size_data,series);
lim = 3.14;

desl = [380 375 395 412 385 406 406];

j=1;
data1 =  data_sv_1_01; 
data2 =  data_sv_1_06(desl(j):end,:); 
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_2_01;
data2 =  data_sv_2_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_3_01; 
data2 =  data_sv_3_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_4_01; 
data2 =  data_sv_4_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_5_01; 
data2 =  data_sv_5_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_6_01; 
data2 =  data_sv_6_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_7_01; 
data2 =  data_sv_7_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end

j=1;i=1;
plot(data_sv_1_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
hold on
plot(data_sv_2_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
plot(data_sv_3_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_4_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_5_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
plot(data_sv_6_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
plot(data_sv_7_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
hold off
title('SV5 Heading Error Comparison');
xlabel ('Time (s)');
ylabel('Error (rad)');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4', 'CSP5');

%% SV5-SV1 Heading Error Comparison - box plot
used_column1 = 7;
used_column2 = sv_head;

size_data = 6180; series = 7;
diff = zeros(size_data,series);
lim = 3.14;

desl = [380 375 395 412 385 406 406];

j=1;
data1 =  data_sv_1_01; 
data2 =  data_sv_1_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_2_01;
data2 =  data_sv_2_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_3_01; 
data2 =  data_sv_3_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_4_01; 
data2 =  data_sv_4_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_5_01; 
data2 =  data_sv_5_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_6_01; 
data2 =  data_sv_6_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end
j=j+1;
data1 =  data_sv_7_01; 
data2 =  data_sv_7_06(desl(j):end,:);
for i=1:size_data-desl(j)
    if (abs(data1(i,used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(i,used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(i,used_column1) + data2(i,used_column2);
    end
end

j=1;i=1;

labels = {'BSP', 'BSP-P', 'CSP1','CSP2', 'CSP3', 'CSP4', 'CSP5'};

figure
boxplot(diff,'Whisker',10, 'Labels', labels)
hold on
title('SC3 - SV5 Heading Error');
xlabel ('Communication Profile');
ylabel('Error (rad)');
xtickangle(45)
hold off

%% ECDF - ERRO DE HEADING
%-------------------------------------------------------------------------
%EMPIRICAL CUMULATIVE DISTRIBUTION FUNCTION
%Ajuste de posição inicial para excluir o erro de posição inicial
%Dados ajustados para o mesmo tamanho. O final é o
%tamanho do menor grupo de dados entre todos os comparados
%-------------------------------------------------------------------------

coluna_used = sv_dist_err;
initial = 0; final = 6180;

[F,X] = ecdf(diff(:,1)) ; stairs(X,F,'--');
hold on
[F,X] = ecdf(diff(:,2)); stairs(X,F,'--');
[F,X] = ecdf(diff(:,3)); stairs(X,F,'-');
[F,X] = ecdf(diff(:,4)); stairs(X,F,'-');
[F,X] = ecdf(diff(:,5)); stairs(X,F,'-.');
[F,X] = ecdf(diff(:,6)); stairs(X,F,'-.');
[F,X] = ecdf(diff(:,7)); stairs(X,F,'-.');
hold off
title('SC3 - SV5 ECDF of Heading Error');
xlabel ('Error (m)');
ylabel('ECDF');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4', 'CSP5');


%%  Heading Comparison - Geometric

used_column1 = 7;
used_column2 = sv_head;
lat = tv_lat;
long = tv_long;

size_data = 6521; series = 6;
diff = zeros(size_data,series);
lim = 3.14;

j=1;
data1 =  data_sv_1_01; 
data2 =  data_sv_1_06;

[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);

vec_dist = zeros(size_data);
vec_ind =zeros(size_data);
%
% for i=1:size_data
%     menor_dist = 999;
%     for j=1:size_data
%        dist = norm(data2(i,tv_lat:tv_long)-data1(:,tv_lat:tv_long)) ;
%        if (dist > menor_dist)
%            menor_dist = dist;
%            ind = j;
%        end
%        vec_dist(i) = menor_dist;
%        vec_ind(i) = ind;
%     end
% 
% end

for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end


%%
j=j+1;
data1 =  data_sv_2_01;
data2 =  data_sv_2_06;
[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);
for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end

j=j+1;
data1 =  data_sv_3_01;
data2 =  data_sv_3_06;
[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);
for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end

j=j+1;
data1 =  data_sv_4_01;
data2 =  data_sv_4_06;
[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);
for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end

j=j+1;
data1 =  data_sv_5_01;
data2 =  data_sv_5_06;
[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);
for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end

j=j+1;
data1 =  data_sv_6_01;
data2 =  data_sv_6_06;
[D,I] = pdist2(data2(:,tv_lat:tv_long),data1(:,tv_lat:tv_long),'euclidean','Smallest',1);
for i=1:size_data
    if (abs(data1(I(i),used_column1) - data2(i,used_column2))<lim)
        diff(i,j)= data1(I(i),used_column1) - data2(i,used_column2);
    else
        diff(i,j)= data1(I(i),used_column1) + data2(i,used_column2);
    end
end


% labels = {'BSP', 'BSP-P', 'CSP1','CSP2', 'CSP3', 'CSP4'};
% 
% figure
% boxplot(diff,'Whisker',20, 'Labels', labels)
% hold on
% title('SV5 Heading Error');
% %xlabel ('Vehicle (s)');
% ylabel('Error (rad)');
% xtickangle(45)
% hold off

j=1;i=1;
plot(data_sv_1_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
hold on
plot(data_sv_2_06(initial:final,sv_time),diff(:,j),':'); j=j+1;
plot(data_sv_3_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_4_06(initial:final,sv_time),diff(:,j),'-'); j=j+1;
plot(data_sv_5_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
plot(data_sv_6_06(initial:final,sv_time),diff(:,j),'-.'); j=j+1;
hold off
title('SV5 Heading Error Comparison');
xlabel ('Time (s)');
ylabel('Error (rad)');
legend('BSP','BSP-P','CSP1','CSP2','CSP3', 'CSP4');

%%



%%
rng('default') % For reproducibility
X = [1 1; 2 2; 3 4]
Y = [1 1; 1 3; 3 5]

[D,I] = pdist2(X,Y,'euclidean','Smallest',1)

%norm(X(2,:)-Y(2,:))






