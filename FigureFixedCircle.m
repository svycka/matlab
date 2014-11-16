classdef FigureFixedCircle < Figure 
    properties
        speed = 5;
        time = 0;
        Y;
        rad;
    end
    
    methods
        function this = FigureFixedCircle(rad, m, cor)
            this.rad = rad;
            this.I = (m*rad^2)/2;
            this.m = m;
            this.phi = cor(3);
            this.cor = cor;
            this.Y = cor(2);
        end
        function addForce(~, spring)
            % todo: jeigu reiks kad sukiotusi nes kitos jegos kaip ir
            % neveikia(itvirtintas taskas)
        end
        function move(this, dt)
            this.cor(1,1) = this.cor(1,1) + dt * this.speed;
            this.time = this.time+dt;
            this.cor(1,2) = this.Y+sin(this.time*this.speed);
        end
        
        function draw(this)
            U = this.U;
            cor = this.cor;
            rad = this.rad;
            
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            rectangle('Position',[xc-rad,yc-rad,2*rad,2*rad],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]);
            % linija orientacijai pavaizduoti:
            plot([xc,xc+rad*cos(phi)], [yc,yc+rad*sin(phi)],'k-'); 
            return
        end
    end
    
end

