% IRIN - Arquitectura Comportamiento
% 
% Diego Dominguez
% Miguel Reino
% Andres Ripoll     
%
%% Borrado de las variables de entorno
clear;
% Su código aquí
%% 1 - Recorrido comportamiento
filename = 'robotPositionComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Recorrido Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'x', 'Trayectoria Comportamiento', [-1.5, 1.5, -1.5, 1.5]);
ylabel('y')
%% 2 - Bateria Comportamiento
filename = 'batteryOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bateria Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Batería Comporatmiento', [-0.1, 500, -0.1, 1.1]);
%% 3 -  Sensor Bateria Comportamiento
filename = 'batteryOutputCompSen';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bateria Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Sensor Batería Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);
%% 4 -  Sensor Avoid Comportamiento
filename = 'avoidOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Avoid Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Avoid Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);
%% 5 -  Sensor Bath Comportamiento
filename = 'bathroomOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Bathroom Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Bathroom Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);
%% 6 -  Sensor Dry Comportamiento
filename = 'DryOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Dry Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Dry Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);
%% 7 -  Sensor Forage Comportarmiento
filename = 'forageOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fig('Forage Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Forage Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);
%% 8 -  Bateria y Sensor Bateria Comportamiento
filename1 = 'batteryOutputCompSen';
filename2 = 'batteryOutputComp';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename1,delimiterIn,headerlinesIn);
B = importdata(filename2,delimiterIn,headerlinesIn);
fig('Bateria Comportamiento');
plot2(A.data(:, 1), A.data(:, 2), 'time[s]', 'Sensor Batería Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);

hold on

plot2(B.data(:, 1), B.data(:, 2), 'time[s]', 'Batería y Sensor Batería Arquitectura Comportamiento', [-0.1, 500, -0.1, 1.1]);

hold off
