function main()
    clear all;clc;clf;
    xmin=-2; xmax=6; ymin=-4; ymax=4;     % paveikslo ribos
    dt = 0.004;
     TT = 10;
    model = World();
    figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on
    for t=0:dt:TT
%         if mod(t, 0.004) == 0,
            title(sprintf('t=%g',t));
            pause(dt); 
            cla;hold on;
            model.draw();
%         end
        model.move(dt);
%         model.draw();
        
        
    end

end