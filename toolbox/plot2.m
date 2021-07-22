function plot2(x, s, xl, t, a)
%PLOT2 Dibuja la señal s con marcadores negros y otros detalles.
%   plot2(n, s, xl, t, axs) dibuja la señal s definida en el intervalo t,
%   titula la gráfica con t y ajusta los ejes con a.
%   Entradas:
%       x - vector de índices temporales
%       s - vector de señal
%       xl - etiqueta para el eje x
%       t - título
%       axs - vector de ajuste de ejes
    plot(x, s)
    xlabel(xl, 'Units', 'normalized', 'Position', [1.05, 0.05])
    title(t, 'FontSize', 10);
    axis(a);
    grid;
end

