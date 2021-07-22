% IRIN - Arquitectura Hibrida
% 
% Diego Dominguez
% Miguel Reino
% Andres Ripoll     
%
%% Borrado de las variables de entorno
clear;
% Su c�digo aqu�
%% 1 - Recorrido hibrido
filename = 'robotPositionHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Recorrido H�brido');
plot2(A.data(:, 2), A.data(:, 3), 'x', 'Trayectoria H�brida', [-1.5, 1.5, -1.5, 1.5]);
ylabel('y')
%% 2 - Bateria Hibrido
filename = 'batteryOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bateria H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Bater�a H�brida', [-0.1, 800, -0.1, 1.1]);
%% 3 -  Sensor Bateria Hibrido
filename = 'batteryOutputHibSen';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bateria H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Sensor Bater�a Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);
%% 4 -  Sensor Avoid Hibrido
filename = 'avoidOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Avoid H�brido');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Avoid Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);
%% 5 -  Sensor Bath Hibrido
filename = 'bathroomOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bathroom H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Bathroom Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);
%% 6 -  Sensor Dry Hibrido
filename = 'DryOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Dry H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Dry Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);
%% 7 -  Sensor Forage Hibrido
filename = 'forageOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Forage H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Forage Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);
%% 8 -  Bateria y Sensor Bateria Hibrido
filename1 = 'batteryOutputHibSen';
filename2 = 'batteryOutputHib';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename1,delimiterIn,headerlinesIn);
B = importdata(filename2,delimiterIn,headerlinesIn);
fig('Bateria H�brida');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Sensor Bater�a Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);

hold on

plot2(B.data(:, 1), B.data(:, 2), 'time[s]', 'Bater�a y Sensor Arquitectura H�brida', [-0.1, 800, -0.1, 1.1]);

hold off
