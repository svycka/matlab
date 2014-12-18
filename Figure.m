classdef (Abstract) Figure < handle
    properties
        g = 9.8; % gravitacijos konstanta
        F = zeros(1,3); % pastovios jegos
        m; % mase
        I; % inercijos momentas
        U=zeros(1,3); % poslinkis
        DU=zeros(1,3); % greitis
        DDU=zeros(1,3); % pagreitis
        phi; % pradinis posukio kampas
        cor; % pradines figuros kordinates
        deltaF = zeros(1,3);
        static=0;
        radius; % spindulys á kurá telpa figûra
    end
    methods (Abstract)
        draw(this)
        getCorners(this)
    end
    methods (Access=public)
        function addForce(this, spring)
            % mazgai prie kuriu jungiama spyruokle laiko momentu t -----
            mazgas1 = spring.rightConnectionPoint();
            mazgas2= spring.leftConnectionPoint();

            % Vektorius is centro, i figuros taska prie kurio prijungta spyruokle (perkeltas i koordinaciu sistemos pradzia) 
            vector = spring.getVector();  
            
            % Apskaiciuojamos spyruokles sukurtos jegos ir jegu momentai, ir
            % pridedama prie pastoviu jegu ----------------------------------------

            L = norm(mazgas1-mazgas2);  % Spyruokles ilgis tam tikru laiko momentu
            deltaL = L - spring.length; % Spyruokles pokytis nuo pradinio ilgio
            T = spring.k*deltaL;        % Spyruikles sukurta jega
            dist=mazgas1-mazgas2;  n=dist/norm(dist); % Spyruokles jegos krypties normales vektorius

            TT = T*n'; % Spyruokles sukurta jega, "paskirstyta"x,y kordinatems

            M_kv=cross([vector;0],[TT'; 0]); % jegos momentas

            % pridedame spyruokles sukurta jega (x,y) asimi ir jegos momenta
            this.deltaF = this.deltaF+[TT +M_kv(3)];    
        end
        
        function addDeltaForce(this, force)
            this.deltaF = this.deltaF+force;    
        end

        function move(this,dt)
            if (this.static == 1)
                return
            end
            F = this.F +this.deltaF;    % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
            this.deltaF = zeros(1,3);
            
            % Pridedamas slopimas proporcingas mazgo greiciui ==============
%             damp_abs = 0.5;
%             F =  F - damp_abs *[this.DU(1),this.DU(2), this.DU(3)];

            % Pagreitis
            this.DDU=this.pagreitis(F, this.m, this.I); % pagreiciai del isoriniu jegu

            %Greitis    
            this.DU=this.DU+dt*this.DDU;      % greiciu ekstrapoliavimas        

            % POSLINKIAI
            this.U=this.U+dt*this.DU;
        end
        
        function center = getCenter(this)
            xc=this.U(1)+this.cor(1);
            yc=this.U(2)+this.cor(2);
            
            center = [xc, yc];
        end
    end
    methods (Access=protected)
        function DDU = pagreitis(~, F, m, I)
            DDU = F ./ [m m I];
        end
    end
    
end

