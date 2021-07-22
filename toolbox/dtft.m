function [X, omega] = dtft(x, N)
%DTFT Calcula N puntos de la transformada de Fourier de la señal x
%   dtft(x, N) calcula numéricamente N puntos en el intervalo [-pi, pi]
%   de la transformada de Fourier de la señal x supuestamente definida
%   en el intervalo [0, length(x)-1].
%   Entradas:
%       x  - vector columna de señal
%       N  - número de puntos de la transformada de Fourier
%   Salida:
%       X - vector columna con la transformada de la señal x
%       omega - vector fila de puntos en los que se calcula la transformada
    omega = linspace(-pi, pi, N)';
    n = 0:length(x)-1;
    X = exp(-1j*omega*n)*x;
end

