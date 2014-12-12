function main()
    clear all;clc;
    xmin=-3; xmax=3; ymin=-4; ymax=4;     % paveikslo ribos
    dt = 0.0005;
     TT = 10;
    model = World();
    figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
    for t=0:dt:TT
        if mod(t, 0.0005) == 0,
            title(sprintf('t=%g',t));
            pause(dt); 
            cla;hold on;
            model.draw();
        end
        model.move(dt);
%         model.draw();
        
        
    end

end