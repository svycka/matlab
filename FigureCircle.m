classdef FigureCircle < Figure
    properties
        rad; % ilgis
    end

    methods
        function this = FigureCircle(rad, m, cor)
            this.rad = rad;
            this.I = (m*rad^2)/2;
            this.m = m;
            this.F = [0 -this.g*m 0];
            this.phi = cor(3);
            this.cor = cor;
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

