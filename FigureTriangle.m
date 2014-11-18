classdef FigureTriangle < Figure
    properties
        a;
        b;
    end
    
    methods
        
        function this = FigureTriangle(a, m, cor)
            this.a = a;
            this.b = a;
            this.I = m*(a^2+this.b^2)/12; % keturkampio inercijos momentas(reiktu trikampio :D)
            this.m = m;
            this.F = [0 -this.g*m 0];
            this.phi = cor(3);
            this.cor = cor;
        end

        function draw(this)
            U = this.U;
            cor = this.cor;
            a = this.a;
            b = this.b;

            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            coord=[-a/2  a/2  0;
                   -b/2 -b/2  b/2;
                     1    1    1  ];   % staciakampis etalonineje padetyje

            T=[cos(phi) -sin(phi) xc;
               sin(phi)  cos(phi) yc;
                  0         0      1 ];      % transformavimo matrica
            coord=T*coord;
            fill(coord(1,:),coord(2,:),[0 0.5 0.7]);
        return 
        end
    end
    
end

