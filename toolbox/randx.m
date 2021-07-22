function x = randx(MIN, MAX)
%RANDX Vector fila de longitud y valores aleatorios.
%   RANDX(MIN, MAX) genera un vector fila de longitud aleatoria (entre MIN
%   y MAX) y valores aleatorios enteros únicos (entre 1-round(MAX/2) y
%   MAX-round(MAX/2)).
assert(MAX >= MIN, 'MAX debe ser mayor o igual que MIN')
x = randperm(MAX, randi([MIN, MAX])) - round(MAX / 2);
end
