function main()
    clear all;clc;
    xmin=-30; xmax=20; ymin=-20; ymax=30;     % paveikslo ribos
    cross1 = Cross(5, 0.3, [-7 2 0], [0 0 0], 1, 'left');
    cross2 = Cross(5, 0.3, [-3 -2 0], [0 0 0], 1, 'right');
    cross3 = Cross(5, 0.3, [2 2 0], [0 0 0], 1, 'left');
    dt = 0.02;
    TT = 30;
    model = Construction();
    figure(1);axis equal;axis([-30,-10,0,15]);hold on, grid on;cla; hold on;
    model.draw();
    %Collision Detection control
    colDetect = CollisionDetection();
    pause();
    figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
    for t=0:dt:TT
        cla;hold on;
        model.move(dt);
        model.draw();
        circleCoords = model.construction{2}.cor(1:2) + model.construction{2}.U(1:2);
        circleRad = model.construction{2}.rad;
        cross1.spinCross(dt);
        colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross1.a, cross1.b, cross1.cor, cross1.U);
        cross2.spinCross(dt);
        cross3.spinCross(dt);
            pause();
        title(sprintf('t=%g',t));
        pause(dt); 
    end
end