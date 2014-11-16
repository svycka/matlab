function main()
    clear all;clc;
    xmin=-30; xmax=30; ymin=-20; ymax=20;     % paveikslo ribos
    figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
    cross1 = Cross(5, 0.3, [-7 2 0], [0 0 0], 1, 'left');
    cross2 = Cross(5, 0.3, [-3 -2 0], [0 0 0], 1, 'right');
    cross3 = Cross(5, 0.3, [2 2 0], [0 0 0], 1, 'left');
    dt = 0.01;
    TT = 10;
    model = Construction();
    for t=0:dt:TT
        figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
        cla; hold on;
        cross1.spinCross(dt);
        cross2.spinCross(dt);
        cross3.spinCross(dt);
        model.move(dt);
        model.draw();
        title(sprintf('t=%g',t));
        pause(dt); 
    end
end