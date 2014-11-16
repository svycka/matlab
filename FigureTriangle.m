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
        
        function addForce(this, spring)
            % mazgas prie kurio jungiama spyruokle laiko momentu t -----
            mazgas1 = spring.rightConnectionPoint();
            L0 = spring.length; % reiks kaþkaip pasiimt ið spyruoklës <------------------------ FIXME
 
            % Apskritimo mazgas prie kurio jungiama spyruokle laiko momentu t -----
%             c_aps = [this.cor(1)+this.U(1)+this.rad*cos(this.phi + this.U(3)); this.cor(2)+this.U(2)+this.rad*sin(this.phi +this.U(3))];  
            c_aps = [this.cor(1)+this.U(1); 
                this.cor(2)+this.U(2)];
            % END -----------------------------------------------------------------
            % Vektorisu is centro, i apskritimo taska prie kurio prijungta spyruokle (perkeltas i koordinaciu sistemos pradzia) 
            c_aps_sp = [this.rad*cos(this.phi+this.U(3)); this.rad*sin(this.phi+this.U(3))];  
            % END -----------------------------------------------------------------

            % Apskaiciuojamos spyruokles sukurtos jegos ir jegu momentai, ir
            % pridedama prie pastoviu jegu ----------------------------------------

            L = norm(c_aps-mazgas1);  % Spyruokles ilgis tam tikru laiko momentu
            deltaL = L -L0;        % Spyruokles pokytis nuo pradinio ilgio
            T = spring.k*deltaL;          % Spyruikles sukurta jega
            dist=c_aps-mazgas1;  n=dist/norm(dist); % Spyruokles jegos krypties normales vektorius

            TT = T*n'; % Spyruokles sukurta jega, "paskirstyta"x,y kordinatems

            M_aps=cross([c_aps_sp;0],[TT'; 0]); % Apskritimo jegos momentas

            this.deltaF = this.deltaF -[TT +M_aps(3)]; % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
        end
        function draw(this)
            xc=this.U(1)+this.cor(1);yc=this.U(2)+this.cor(2);
%             fill([this.cor(5,1) this.cor(5,1)-0.5 this.cor(5,1)+0.5], [this.cor(5,2)-0.5 this.cor(5,2)+0.5 this.cor(5,2)+0.5], 1);
            fill([xc xc-0.5 xc+0.5], [yc-0.5 yc+0.5 yc+0.5], 1);
        return 
        end
    end
    
end

