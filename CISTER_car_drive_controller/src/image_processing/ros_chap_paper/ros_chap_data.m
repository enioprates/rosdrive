%% NETWORK PERFORMANCE - TRACK ANALYSIS

%% ajustando formato de dados
%format long;
%format compact;
format longG;
%close all;
%clear;
clc;

text = 'config!'

%% Loading Data 
%(Change the "\" for "/" if you are using LINUX!)

%DEFINE THE NUMBER OF VEHICLES
n_cars = 1;

%DATA 7--------------------------------------------

filename = 'curve_s_20_005_00_00/log_position_SV_car1.csv';
data_sv_s_01_01 = load(filename);

filename = 'curve_s_20_02_00_00/log_position_SV_car1.csv';
data_sv_s_02_01 = load(filename);

filename = 'curve_s_20_025_00_00/log_position_SV_car1.csv';
data_sv_s_03_01 = load(filename);

filename = 'curve_s_20_03_00_00/log_position_SV_car1.csv';
data_sv_s_04_01 = load(filename);

filename = 'curve_s_20_03_01_00/log_position_SV_car1.csv';
data_sv_s_05_01 = load(filename);

filename = 'curve_s_20_05_00_00/log_position_SV_car1.csv';
data_sv_s_06_01 = load(filename);

filename = 'curve_s_20_10_01_00/log_position_SV_car1.csv';
data_sv_s_07_01 = load(filename);

filename = 'curve_s_20_108_0216_0135/log_position_SV_car1.csv';
data_sv_s_08_01 = load(filename);

filename = 'curve_s_20_108_0416_0135/log_position_SV_car1.csv';
data_sv_s_09_01 = load(filename);

filename = 'curve_s_20_15_00_00/log_position_SV_car1.csv';
data_sv_s_10_01 = load(filename);

filename = 'curve_s_20_16_00_00/log_position_SV_car1.csv';
data_sv_s_11_01 = load(filename);

filename = 'curve_s_20_17_00_00/log_position_SV_car1.csv';
data_sv_s_12_01 = load(filename);

filename = 'curve_s_20_18_00_00/log_position_SV_car1.csv';
data_sv_s_13_01 = load(filename);

%----------------------------------------------------------

filename = 'curve_t_13_10_00_00/log_position_SV_car1.csv';
data_sv_t_13_10_00_00 = load(filename);

filename = 'curve_t_15_10_00_00/log_position_SV_car1.csv';
data_sv_t_15_10_00_00 = load(filename);

filename = 'curve_t_15_10_00_005/log_position_SV_car1.csv';
data_sv_t_15_10_00_005 = load(filename);

filename = 'curve_t_15_10_00_010/log_position_SV_car1.csv';
data_sv_t_15_10_00_010 = load(filename);

filename = 'curve_t_15_10_01_010/log_position_SV_car1.csv';
data_sv_t_15_10_01_010 = load(filename);

filename = 'curve_t_15_10_005_00/log_position_SV_car1.csv';
data_sv_t_15_10_005_00 = load(filename);

filename = 'curve_t_15_10_005_005/log_position_SV_car1.csv';
data_sv_t_15_10_005_005 = load(filename);

filename = 'curve_t_15_10_010_00/log_position_SV_car1.csv';
data_sv_t_15_10_010_00 = load(filename);

filename = 'curve_t_15_12_00_00/log_position_SV_car1.csv';
data_sv_t_04_01 = load(filename);

filename = 'curve_t_15_20_00_00/log_position_SV_car1.csv';
data_sv_t_05_01 = load(filename);

%--------------------------------------------

filename = 'curve_tf_13_10_000_000/log_position_SV_car1.csv';
data_sv_tf_13_10_000_000 = load(filename);

filename = 'curve_tf_15_10_000_000/log_position_SV_car1.csv';
data_sv_tf_15_10_000_000 = load(filename);

filename = 'curve_tf_15_10_000_005/log_position_SV_car1.csv';
data_sv_tf_15_10_000_005 = load(filename);

filename = 'curve_tf_15_10_000_010/log_position_SV_car1.csv';
data_sv_tf_15_10_000_010 = load(filename);

%--------------------------------------------

filename = 'curve_oa_13_10_00_00/log_position_SV_car1.csv';
curve_oa_13_10_00_00 = load(filename);

filename = 'curve_oa_15_10_00_00/log_position_SV_car1.csv';
curve_oa_15_10_00_00 = load(filename);

filename = 'curve_oa_15_10_00_05/log_position_SV_car1.csv';
curve_oa_15_10_00_05 = load(filename);

filename = 'curve_oa_15_10_00_10/log_position_SV_car1.csv';
curve_oa_15_10_00_10 = load(filename);

%--------------------------------------------

filename = 'curve_2v_13_12/log_position_SV_car1.csv';
curve_2v_13_12_1 = load(filename);

filename = 'curve_2v_13_12/log_position_SV_car2.csv';
curve_2v_13_12_2 = load(filename);


filename = 'curve_2v_14_12/log_position_SV_car1.csv';
curve_2v_14_12_1 = load(filename);

filename = 'curve_2v_14_12/log_position_SV_car2.csv';
curve_2v_14_12_2 = load(filename);


filename = 'curve_2v_15_12/log_position_SV_car1.csv';
curve_2v_15_12_1 = load(filename);

filename = 'curve_2v_15_12/log_position_SV_car2.csv';
curve_2v_15_12_2 = load(filename);


filename = 'curve_2v_16_12/log_position_SV_car1.csv';
curve_2v_16_12_1 = load(filename);

filename = 'curve_2v_16_12/log_position_SV_car2.csv';
curve_2v_16_12_2 = load(filename);


filename = 'curve_2v_20_12/log_position_SV_car1.csv';
curve_2v_20_12_1 = load(filename);

filename = 'curve_2v_20_12/log_position_SV_car2.csv';
curve_2v_20_12_2 = load(filename);




text = 'LOAD Graph!'












