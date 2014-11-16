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
    end
    methods (Abstract)
        draw(this) 
        addForce(this, spring)
    end
    methods (Access=public)
        function move(this,dt)
            F = this.F +this.deltaF;    % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
            this.deltaF = zeros(1,3);
            
            % Pridedamas slopimas proporcingas mazgo greiciui ==============
            damp_abs = 1;
            F =  F - damp_abs *[this.DU(1),this.DU(2), this.DU(3)];

            % Pagreitis
            this.DDU=this.pagreitis(F, this.m, this.I); % pagreiciai del isoriniu jegu

            %Greitis    
            this.DU=this.DU+dt*this.DDU;      % greiciu ekstrapoliavimas        

            % POSLINKIAI
            this.U=this.U+dt*this.DU;
        end
    end
    methods (Access=protected)
        function DDU = pagreitis(~, F, m, I)
            DDU = F ./ [m m I];
        end
    end
    
end

