function x = sinusoid(w0, phi, n, n0)
%SINUSOID Función sinusoide
% x = sinusoid(w0, phi, n, n0) calcula la señal cos(w0(n-n0)+phi) en el intervalo n dado.
% Los parámetros de entrada son:
%   w0 - escalar real frecuencia de la sinusoide
%   phi - escalar real fase de la sinusoide
%   n  - vector de índices enteros en los que se calcula la señal
%   n0 - escalar entero que indica el retardo
    x = cos(w0*(n - n0)+phi)';
end

