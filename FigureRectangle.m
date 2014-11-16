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
            this.len = 0; % kolkas 0 nes centre bet reikts paskai�iuot
        end
        function addForce(this, spring)
            % mazgas prie kurio jungiama spyruokle laiko momentu t -----
            mazgas1 = spring.rightConnectionPoint();
            L0 = spring.length; % reiks ka�kaip pasiimt i� spyruokl�s <------------------------ FIXME

            % Kvadrato kampas prie kurio jungiama spyruokle -----------------------
            % alpha, len - kintamieji apskaiciuoti pries sk. integravima
            G=this.U(1:2)+this.cor(1:2);    % Kvadrato centro kordinates
            phi=this.U(3);                % Kvadrato posukio kampas;
            % Vektorius i� centro i itvirtinimo taska
            S= [cos(phi+this.alpha) -sin(phi+this.alpha) ;
                     sin(phi+this.alpha)  cos(phi+this.alpha)]*[this.len;0];      
            c_kv= S+G';  

            % Apskaiciuojamos spyruokles sukurtos jegos ir jegu momentai, ir
            % pridedama prie pastoviu jegu ----------------------------------------

            L = norm(mazgas1-c_kv);  % Spyruokles ilgis tam tikru laiko momentu
            deltaL = L -L0;        % Spyruokles pokytis nuo pradinio ilgio
            T = spring.k*deltaL;          % Spyruikles sukurta jega
            dist=mazgas1-c_kv;  n=dist/norm(dist); % Spyruokles jegos krypties normales vektorius

            TT = T*n'; % Spyruokles sukurta jega, "paskirstyta"x,y kordinatems

            M_kv=cross([S;0],[TT'; 0]); % Staciakampio jegos momentas

            this.deltaF = this.deltaF+[TT +M_kv(3)];    % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
        end
        
        function draw(this)
            U = this.U;
            cor = this.cor;
            a = this.a;
            b = this.b;
            
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            coord=[-a/2  a/2  a/2 -a/2;
                   -b/2 -b/2  b/2  b/2;
                     1    1    1    1  ];   % staciakampis etalonineje padetyje
            T=[cos(phi) -sin(phi) xc;
               sin(phi)  cos(phi) yc;
                  0         0      1 ];      % transformavimo matrica
            coord=T*coord;
            fill(coord(1,:),coord(2,:),[0 0.5 0.7]);
            return
        end
    end
    
end
