function test = isalmostequal(actual, expected, type, tol)
%ISALMOSTEQUAL Comprueba si dos valores son casi iguales.
% ISALMOSTEQUAL(actual, expected) comprueba si abs(actual-expected) <= eps.
% Es equivalente a ISALMOSTEQUAL(actual, expected, 'AbsTol', eps).
% Si TYPE = 'RelTol', la comprobación es abs(actual-expected) <= tol*abs(expected).
%
% Parámetros de entrada:
%   actual: valores actuales
%   expected: valores esperados
%   tipo de tolerancia: 'AbsTol' absoluta, 'RelTol' relativa
%   tol: tolerancia
if nargin == 2
    tol = eps;
    type = 'AbsTol';
end
diff = abs(actual - expected);
if strcmp(type, 'Abstol')
    test = all(diff <= tol);
else
    test = all(diff <= tol*abs(expected));
end
end

