classdef Construction < handle
    properties
        g = 8.8;
        k = 1000;
        m = 2;
        I_kv; %inercijos momentas
        
        ind = [1 2
               1 3
               1 4
               2 3
               3 2
               4 3
               3 4
               5 3
               6 4
               7 4
               6 7]; %elementai
        cor = [-17 7 0
                -22 9 0
                -22 7 0
                -22 5 0
                -26 7 0
                -28 5 0
                -28 2 0]; %mazgu koordinates
        IS = [1 1  0 0  0 0  0 0  0 0  0 0  0 0]; %mazgu itvirtinimai
        rad = [-1.5 -1.5
               -0.5 -0.5
               -1.5 -0.5
               -1.5 -0.5
               -0.5 0.5
               -0.5 0.5
               -0.5 0.5
            ]; %nuokrypis
        G;
        phi = 0;
        alpha = 0;%pi/2+asin((3/2) / sqrt((3/2)^2+(1/2)^2));
        len = 0;%sqrt((3/2)^2+(1/2)^2);
        S;
        nmz = 7;
        time = 0;
        U = zeros(14, 3);
        DU = zeros(14, 3);
        DDU = zeros(14, 3);
        deltaY = 0;
        L0;
    end
    
    methods
        function this = Construction()
            this.deltaY = this.cor(1,2);
            this.I_kv = this.m*(3^2+1^2)/12;
            this.G = this.cor(3,1:2);
            this.S = [cos(this.phi+this.alpha) -sin(this.phi+this.alpha) ;
                sin(this.phi+this.alpha) cos(this.phi+this.alpha)]*[this.len;0];
this.S = [0 0;0 0]*[this.len;0];
            this.L0 = norm(this.cor(1,1)-this.cor(3,1));
        end
        function create(this)
            %mazgai
            rectangle('Position',[this.cor(1,1)+this.rad(1,1), this.cor(1,2)+this.rad(1,2),3,3],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
            rectangle('Position',[this.cor(2,1)+this.rad(2,1), this.cor(2,2)+this.rad(2,2),1,1],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
%             rectangle('Position',[this.cor(3,1)+this.rad(3,1),this.cor(3,2)+this.rad(3,2),3,1]);
            this.braizyti_staciakampi(this.U(3,:), this.cor(3,:), 3, 1);
            rectangle('Position',[this.cor(4,1)+this.rad(4,1),this.cor(4,2)+this.rad(4,2),3,1]);
%             p1=[this.cor(5,1)   this.cor(5,2)-0.5];
%             p2=[this.cor(5,1)-0.5 this.cor(5,2)+0.5]; 
%             p3=[this.cor(5,1)+0.5 this.cor(5,2)+0.5];
%             x = [p1(1) p2(1) p3(1)]; 
%             y = [p1(2) p2(2) p3(2)]; 
            fill([this.cor(5,1) this.cor(5,1)-0.5 this.cor(5,1)+0.5], [this.cor(5,2)-0.5 this.cor(5,2)+0.5 this.cor(5,2)+0.5], 1);
            fill([this.cor(6,1) this.cor(6,1)-0.5 this.cor(6,1)+0.5], [this.cor(6,2)-0.5 this.cor(6,2)+0.5 this.cor(6,2)+0.5], 1);
            fill([this.cor(7,1) this.cor(7,1)-0.5 this.cor(7,1)+0.5], [this.cor(7,2)-0.5 this.cor(7,2)+0.5 this.cor(7,2)+0.5], 1);
            %elementai
            nme = length(this.ind); %elementu kiekis
            %braizom elementus
            for i=1:nme,
                cxf= this.rad(this.ind(i,1),1);
                cyf= this.rad(this.ind(i,1),2);
                cxt= this.rad(this.ind(i,2),1);
                cyt= this.rad(this.ind(i,2),2);
                
%                 line([this.cor(this.ind(i,1),1)+this.U(this.ind(i,1),1) this.cor(this.ind(i,2),1)],[this.cor(this.ind(i,1),2)+this.U(this.ind(i,1),2) this.cor(this.ind(i,2),2)]);
            end
            line([this.cor(3,1)+this.U(3,1) this.cor(1,1)],[this.cor(3,2)+this.U(3,2) this.cor(1,2)]);

        end
        
        function moveStatic(this, dt, speedX)
            this.cor(1,1) = this.cor(1,1) + dt * speedX;
            this.time = this.time+dt;
            this.cor(1,2) = this.deltaY+sin(this.time*speedX);
            return
        end
        
        function moveSquare(this,dt)
            % Apskritimo mazgas prie kurio jungiama spyruokle laiko momentu t -----
            c_aps = [this.cor(1,1)+this.U(1,1); 
                this.cor(1,2)+this.U(1,2)];

            this.G = this.cor(3,1:2) + this.U(3,1:2);
            this.S = [cos(this.phi+this.alpha) -sin(this.phi+this.alpha) ;
                sin(this.phi+this.alpha) cos(this.phi+this.alpha)]*[this.len;0];
            c_kv = this.S+this.G';

            
            
            
            L = norm(c_aps-c_kv);
            deltaL = L-this.L0;
            T = this.k*deltaL;
            dist = c_aps-c_kv;
            n = dist/norm(dist);
            TT = T*n';
            M_kv = cross([this.S;0],[TT';0]);
            T_kv = 0 + [TT M_kv(3)];
            this.DDU(3,:) = this.pagreitis(T_kv, this.m, this.I_kv); %patikrinti 
            this.DU(3,:) = this.DU(3,:)+dt*this.DDU(3,:);
            this.U(3,:) = this.U(3,:)+dt*this.DU(3,:);
            this.DDU(3,:)
            return
        end
        
        function DDU = pagreitis(~, F, m, I)
            DDU = F ./ [m m I];
        end
        function braizyti_staciakampi(this,U,cor,a, b)
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

