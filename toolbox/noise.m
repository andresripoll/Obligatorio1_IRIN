function r = noise(N, mu, sigma, type)
%NOISE Generador de ruido blanco 
%   r = noise(N, mu, sigma, type) genera un vector columna con N muestras
%   de ruido blanco de media mu y desviación típica sigma. La distribución
%   puede ser uniforme o gaussiana.
%   Entradas:
%       N - número de muestras de ruido
%       mu - valor medio
%       sigma - desviación típica
%       type - tipo de distribución; puede ser "uniform" o "gaussian"
%   Salidas:
%       r - vector columna de ruido
%
if type == "uniform"
    a = mu - sqrt(3)*sigma;
    b = mu + sqrt(3)*sigma;
    r = a + (b-a)*rand(N, 1);
elseif type == "gaussian"
    r = mu + sigma*randn(N, 1);
else
    error('Distribución descononocida');
end
end