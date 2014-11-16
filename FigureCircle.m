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
        function addForce(this, spring)
            % mazgas prie kurio jungiama spyruokle laiko momentu t -----
            mazgas1 = spring.rightConnectionPoint();
 
            % Apskritimo mazgas prie kurio jungiama spyruokle laiko momentu t -----
            c_aps = spring.leftConnectionPoint();
            % END -----------------------------------------------------------------
            
            % Vektorius is centro, i apskritimo taska prie kurio prijungta spyruokle (perkeltas i koordinaciu sistemos pradzia) 
            vector = spring.getVector();  
            % END -----------------------------------------------------------------

            % Apskaiciuojamos spyruokles sukurtos jegos ir jegu momentai, ir
            % pridedama prie pastoviu jegu ----------------------------------------

            L = norm(c_aps-mazgas1);  % Spyruokles ilgis tam tikru laiko momentu
            deltaL = L -spring.length;        % Spyruokles pokytis nuo pradinio ilgio
            T = spring.k*deltaL;          % Spyruikles sukurta jega
            dist=c_aps-mazgas1;  n=dist/norm(dist); % Spyruokles jegos krypties normales vektorius

            TT = T*n'; % Spyruokles sukurta jega, "paskirstyta"x,y kordinatems

            M_aps=cross([vector;0],[TT'; 0]); % Apskritimo jegos momentas

            this.deltaF = this.deltaF -[TT +M_aps(3)]; % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
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

