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
        rectCoords = model.construction{3}.cor(1:3);
        rectA = model.construction{3}.a;
        rectB = model.construction{3}.b;
        rectU = model.construction{3}.U;
        triCoords = model.construction{6}.cor(1:3);
        triA = model.construction{5}.a;
        triB = model.construction{5}.b;
        triU = model.construction{5}.U;
        cross1.spinCross(dt);
%         colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross1.a, cross1.b, cross1.cor, cross1.U);
%         colDetect.detectRectangleCollision(cross1.cor, cross1.a, cross1.b, rectCoords, rectA, rectB, rectU);
        colDetect.detectTriangleCollision(cross1.cor, cross1.b, cross1.a, triCoords, triA, triB, triU);
         colDetect.detectTriangleCollision(cross1.cor, cross1.a, cross1.b, triCoords, triA, triB, triU);
        cross2.spinCross(dt);
%         colDetect.detectTriangleCollision(cross2.cor, cross2.b, cross2.a, triCoords, triA, triB, triU);
%         colDetect.detectTriangleCollision(cross2.cor, cross2.a, cross2.b, triCoords, triA, triB, triU);
        cross3.spinCross(dt);
%         colDetect.detectTriangleCollision(cross3.cor, cross3.b, cross3.a, triCoords, triA, triB, triU);
%         colDetect.detectTriangleCollision(cross3.cor, cross3.a, cross3.b, triCoords, triA, triB, triU);
            pause();
        title(sprintf('t=%g',t));
        pause(dt); 
    end
end