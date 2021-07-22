function [x, n] = idtft(X, M)
%DTFT Calcula M puntos de la señal x con transformada de Fourier X
%   idtft(x) calcula numéricamente M puntos de la señal x con transformada
%   de Fourier X en el intervalo [0, M-1].
%   Entradas:
%       X  - vector columna de la transformada de Fourier de la señal
%       M  - número de puntos de la señal
%   Salida:
%       x - vector columna con la señal x
%       n - vector fila de índices de la señal x
    omega = linspace(-pi, pi, length(X))';
    n = 0:M-1;
    Xk = repmat(X, 1, length(n)).*exp(1j*omega*n);
    x = real(trapz(omega, Xk))'/(2*pi);    
end
