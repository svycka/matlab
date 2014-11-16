function main()
    clear all;clc;
    xmin=-30; xmax=100; ymin=-20; ymax=30;     % paveikslo ribos
    cross1 = Cross(5, 0.3, [-7 2 0], [0 0 0], 1, 'left');
    cross2 = Cross(5, 0.3, [-3 -2 0], [0 0 0], 1, 'right');
    cross3 = Cross(5, 0.3, [2 2 0], [0 0 0], 1, 'left');
    dt = 0.02;
    TT = 30;
    model = Construction();
    figure(1);axis equal;axis([-30,-10,0,15]);hold on, grid on;cla; hold on;
    model.draw();
    pause();
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