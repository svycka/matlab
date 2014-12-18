classdef FigureRectangle < Figure
    properties
        a; % ilgis
        b; % plotis
        alpha = pi/2; % Pradinis posukio kampas i itvirtinimo mazga
        len; % Atkarpos is centro, i itvirtinimo taska ilgis
    end

    methods
        function this = FigureRectangle(a, b, m, cor)
            this.a = a;
            this.b = b;
            this.m = m;
            this.F = [0 -this.g*m 0];
            this.I = m*(a^2+b^2)/12;
            this.phi = cor(3);
            this.cor = cor; % [x y posukio_kampas]
            this.radius = sqrt(a^2+b^2)/2;
        end

        function draw(this)
            coord=this.getCorners();
            fill(coord(1,:),coord(2,:),[0 0.5 0.7]);
            return
        end

        function coord = getCorners(this)
            U = this.U;
            cor = this.cor;
            a = this.a;
            b = this.b;
            
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            coord=[-a/2 -a/2   a/2  a/2; 
                   -b/2  b/2   b/2  -b/2;
                     1    1    1     1  ];   % staciakampis etalonineje padetyje
            T=[cos(phi) -sin(phi) xc;
               sin(phi)  cos(phi) yc;
                  0         0      1 ];      % transformavimo matrica
            coord=T*coord;
        end
    end
    
end

