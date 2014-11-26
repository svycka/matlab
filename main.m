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
        for i=3:4
            rectCoords(i-2,1:3) = model.construction{i}.cor(1:3);
            rectA(i-2) = model.construction{i}.a;
            rectB(i-2) = model.construction{i}.b;
            rectU(i-2,1:3) = model.construction{i}.U;
        end
        for i=5:7
            triCoords(i-4, 1:3) = model.construction{i}.cor(1:3);
            triA(i-4) = model.construction{i}.a;
            triB(i-4) = model.construction{i}.b;
            triU(i-4, 1:3) = model.construction{i}.U;   
        end                       
        cross1.spinCross(dt);
%         for j=1:3
%             colDetect.detectTriangleCollision(cross1.cor, cross1.b, cross1.a, cross1.U, cross1.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%             colDetect.detectTriangleCollision(cross1.cor, cross1.a, cross1.b, cross1.U, cross1.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%         end
%          for j=1:2
%              colDetect.detectRectangleCollision(cross1.cor, cross1.a, cross1.b, cross1.U, cross1.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%              colDetect.detectRectangleCollision(cross1.cor, cross1.b, cross1.a, cross1.U, cross1.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%          end
         colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross1.a, cross1.b, cross1.cor, cross1.U);
        colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross1.b, cross1.a, cross1.cor, cross1.U);
         cross2.spinCross(dt);
%         for j=1:3
%             colDetect.detectTriangleCollision(cross2.cor, cross2.b, cross2.a, cross2.U, cross2.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%             colDetect.detectTriangleCollision(cross2.cor, cross2.a, cross2.b, cross2.U, cross2.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%         end
%          for j=1:2
%              colDetect.detectRectangleCollision(cross2.cor, cross2.a, cross2.b, cross2.U, cross2.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%              colDetect.detectRectangleCollision(cross2.cor, cross2.b, cross2.a, cross2.U, cross2.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%          end
colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross2.a, cross2.b, cross2.cor, cross2.U);
        colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross2.b, cross2.a, cross2.cor, cross2.U);
cross3.spinCross(dt);
%         for j=1:3
%             colDetect.detectTriangleCollision(cross3.cor, cross3.b, cross3.a, cross3.U, cross3.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%             colDetect.detectTriangleCollision(cross3.cor, cross3.a, cross3.b, cross3.U, cross3.direction, triCoords(j,1:3), triA(j), triB(j), triU(j,1:3));
%         end
%          for j=1:2
%              colDetect.detectRectangleCollision(cross3.cor, cross3.a, cross3.b, cross3.U, cross3.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%              colDetect.detectRectangleCollision(cross3.cor, cross3.b, cross3.a, cross3.U, cross3.direction, rectCoords(j,1:3), rectA(j), rectB(j), rectU(j,1:3));
%          end
colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross2.a, cross3.b, cross3.cor, cross3.U);
colDetect.detectCircleCollision(circleCoords(1), circleCoords(2), circleRad, cross3.b, cross3.a, cross3.cor, cross3.U);
         pause();
        title(sprintf('t=%g',t));
        pause(dt); 
    end
end