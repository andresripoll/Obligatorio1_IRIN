function xt0 = perext(x, M)
%PEREXT Extensión periódica de x
%   PEREXT(x, M) devuelve un periodo de la extensión periódica de periodo M
%   de la señal x.
%   Entradas:
%     x: vector columna de señal
%     M: periodo de la extensión periódica
%   Salidas:
%     xt0: vector columna con un periodo de la extensión periódica de x 
L = length(x);
if L <= M
    xt0 = zeropad(x, M);
else
    xt0 = sum(buffer(x, M), 2);
end
end
