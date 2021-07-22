function [] = fig(name)
%FIG Crea o activa una ventana gráfica de nombre 'name'.
%   h = fig(name) activa la ventana gráfica de nombre 'name'. Si no existe, 
%   se crea y se le asigna el nombre.
    h = findobj('Name', name);
    if isempty(h)
        figure('Name', name);
    else
        figure(h)
    end
end

