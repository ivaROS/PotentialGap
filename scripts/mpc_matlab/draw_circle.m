function [xunit, yunit] = draw_circle(x,y,r,thick, color)
    hold on;
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color, 'LineWidth',thick);
end
