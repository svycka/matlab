function main()
    clear all;clc;
    xmin=-3; xmax=3; ymin=-4; ymax=4;     % paveikslo ribos
    dt = 0.002;
    TT = 10;
    model = World();
    figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
    for t=0:dt:TT
        model.draw();
        model.move(dt);
%         model.draw();
        
        title(sprintf('t=%g',t));
        pause(dt); 
        cla;hold on;
    end

end