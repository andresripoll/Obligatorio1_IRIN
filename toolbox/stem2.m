function stem2(n, s, xl, t, axs)
%STEM2 Dibuja la señal s con marcadores negros y otros detalles.
%   stem2(n, s, xl, t, axs) dibuja la señal s definida en el intervalo n,
%   titula la gráfica con t y ajusta los ejes con axs.
%   Entradas:
%       n - vector de índices temporales
%       s - vector de señal
%       xl - etiqueta para el eje x
%       t - título
%       axs - vector de ajuste de ejes
stem(n, s, 'k', 'MarkerfaceColor', 'k')
xlabel(xl, 'Units', 'normalized', 'Position', [1.025, 0.15])
title(t, 'FontSize', 10);
axis(axs);
grid('on');
end
