classdef FigureCircle < Figure
    properties
        rad; % ilgis
    end

    methods
        function this = FigureCircle(rad, m, cor)
            this.rad = rad;
            this.radius = rad;
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
            coord = this.getCorners();
            plot(coord(1,:), coord(2,:),'k-');
            return
        end
        
        function coord = getCorners(this)
            U = this.U;
            cor = this.cor;
            rad = this.rad;
            
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            coord=[0    rad;
                   0    0;
                   1    1];   % staciakampis etalonineje padetyje
            T=[cos(phi) -sin(phi) xc;
               sin(phi)  cos(phi) yc;
                  0         0      1 ];      % transformavimo matrica
            coord=T*coord;
        end
    end
    
end

