function [B, A] = samelength(B, A)
%SAMELENGTH Fuerza que los vectores de entrada tengan la misma longitud
%   [B, A] = SAMELENGTH(B, A) devuelve los vectores B, A con la misma
%   longitud; si alguno de ellos es menor se añaden los ceros necesarios.
%   Los vectores pueden ser ambos fila, ambos columna o uno fila y otro
%   columna.
nb = length(B);
na = length(A);
addA = zeros(1, nb-na);
A(end+1:end+length(addA)) = addA;
addB = zeros(1, na-nb);
B(end+1:end+length(addB)) = addB;
end

