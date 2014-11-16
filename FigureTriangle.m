classdef FigureTriangle < Figure
    properties
        rad;
    end
    
    methods
        
        function this = FigureTriangle(rad, m, cor)
            this.rad = rad;
            this.I = (m*rad^2)/2; % apskritimo inercijos momentas(reiktu trikampio :D)
            this.m = m;
            this.F = [0 -this.g*m 0];
            this.phi = cor(3);
            this.cor = cor;
        end

        function draw(this)
            xc=this.U(1)+this.cor(1);yc=this.U(2)+this.cor(2);
            fill([xc xc-0.5 xc+0.5], [yc-0.5 yc+0.5 yc+0.5], 1);
        return 
        end
    end
    
end

