function [xq, delta] = quant(Xm, N, x)
%QUANT Cuantifica la señal x[n]
%   xq = QUANT(Xm, N, x) cuantifica los valores del vector x con un
%   cuantificador uniforme de amplitud máxima Xm y N bits.
%   Entradas:
%       Xm - Valor a fondo de escala
%       N  - Número de bits
%       x  - señal a cuantificar
%   [xq, delta] = QUANT(...) devuelve además el paso de cuantificación.

% Paso de cuantificación
delta = 2*Xm/2^N;
% Cuantificación
xq = delta*round(x/delta);
% Saturación
xqmax = Xm*(1-2^(-N+1)); 
xq(x >= xqmax) = xqmax;
xqmin = -Xm;
xq(x <= xqmin) = xqmin;
end

