classdef Construction < handle
    properties
        g = 9.8;
        k = 1000;
        m = 2;
        I_kv; %inercijos momentas
        I_aps; % inercijos momentai
        F; % pastovios jëgos
        ind = [1 2
               1 3
               1 4
               2 3
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
        L0aps;
        L0k2;
        L0tr1;
        phi_aps;
    end
    
    methods
        function this = Construction()
            this.deltaY = this.cor(1,2);
            this.I_kv = this.m*(3^2+1^2)/12;
            this.F = [0 0 0
                0 0 0
                0 -this.g*this.m*10 0
                0 0 0
                0 0 0
                0 0 0
                0 0 0
                0 0 0
                0 0 0
                0 0 0
                0 0 0];
            this.G = this.cor(3,1:2);
            this.S = [cos(this.phi+this.alpha) -sin(this.phi+this.alpha) ;
                sin(this.phi+this.alpha) cos(this.phi+this.alpha)]*[this.len;0];
this.S = [0 0;0 0]*[this.len;0];
            this.L0 = norm(this.cor(1,1)-this.cor(3,1));
            this.L0k2 = norm(this.cor(1,1:2)-this.cor(4,1:2));
            this.I_aps = (this.m * 1 ^ 2)/2;
            this.L0aps = norm(this.cor(1,1:2)-this.cor(2,1:2));
             this.L0tr1 = norm(this.cor(3,1:2)-this.cor(5,1:2));
            this.phi_aps = pi/2;
        end
        function create(this)
            %mazgai
            rectangle('Position',[this.cor(1,1)+this.rad(1,1), this.cor(1,2)+this.rad(1,2),3,3],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
            %rectangle('Position',[this.cor(2,1)+this.rad(2,1), this.cor(2,2)+this.rad(2,2),1,1],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
%             rectangle('Position',[this.cor(3,1)+this.rad(3,1),this.cor(3,2)+this.rad(3,2),3,1]);
            this.braizyti_staciakampi(this.U(3,:), this.cor(3,:), 3, 1);
            this.braizyti_staciakampi(this.U(4,:), this.cor(4,:), 3, 1);
            this.braizyti_apskritima(this.U(2,:), this.cor(2,:), 1);
            %rectangle('Position',[this.cor(4,1)+this.rad(4,1),this.cor(4,2)+this.rad(4,2),3,1]);
%             p1=[this.cor(5,1)   this.cor(5,2)-0.5];
%             p2=[this.cor(5,1)-0.5 this.cor(5,2)+0.5]; 
%             p3=[this.cor(5,1)+0.5 this.cor(5,2)+0.5];
%             x = [p1(1) p2(1) p3(1)]; 
%             y = [p1(2) p2(2) p3(2)]; 
            this.braizyti_trikampi(this.U(5,:),this.cor(5,:),1); 
            this.braizyti_trikampi(this.U(6,:),this.cor(6,:),1); 
            this.braizyti_trikampi(this.U(7,:),this.cor(7,:),1); 
            %fill([this.cor(5,1) this.cor(5,1)-0.5 this.cor(5,1)+0.5], [this.cor(5,2)-0.5 this.cor(5,2)+0.5 this.cor(5,2)+0.5], 1);
%             fill([this.cor(6,1) this.cor(6,1)-0.5 this.cor(6,1)+0.5], [this.cor(6,2)-0.5 this.cor(6,2)+0.5 this.cor(6,2)+0.5], 1);
%             fill([this.cor(7,1) this.cor(7,1)-0.5 this.cor(7,1)+0.5], [this.cor(7,2)-0.5 this.cor(7,2)+0.5 this.cor(7,2)+0.5], 1);
            %elementai
            nme = length(this.ind); %elementu kiekis
            %braizom elementus
            for i=1:nme,
                cxf= this.rad(this.ind(i,1),1);
                cyf= this.rad(this.ind(i,1),2);
                cxt= this.rad(this.ind(i,2),1);
                cyt= this.rad(this.ind(i,2),2);
                line([this.cor(this.ind(i,2),1)+this.U(this.ind(i,2),1) this.cor(this.ind(i,1),1)+this.U(this.ind(i,1),1)],[this.cor(this.ind(i,2),2)+this.U(this.ind(i,2),2) this.cor(this.ind(i,1),2)+this.U(this.ind(i,1),2)]);
            end

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
            deltaL = L-this.L0k2;
            T = this.k*deltaL;
            dist = c_aps-c_kv;
            n = dist/norm(dist);
            TT = T*n';
            M_kv = cross([this.S;0],[TT';0]);
            T_kv = this.F(3,:) + [TT +M_kv(3)];
            % pagreitis
            this.DDU(3,:) = this.pagreitis(T_kv, this.m, this.I_kv); %patikrinti 
            % greitis
            this.DU(3,:) = this.DU(3,:)+dt*this.DDU(3,:);
            % poslinkis
            this.U(3,:) = this.U(3,:)+dt*this.DU(3,:);
            

            % pagreitis
            this.DDU(4,:) = this.pagreitis(T_kv, this.m, this.I_kv); %patikrinti 
            % greitis
            this.DU(4,:) = this.DU(4,:)+dt*this.DDU(4,:);
            % poslinkis
            this.U(4,:) = this.U(4,:)+dt*this.DU(4,:);
            
            
            
            
            
            
            
            
            
            
            
            
            
            %APSKRITIMAS
            c_aps = [this.cor(3,1)+this.U(3,1); 
                this.cor(3,2)+this.U(3,2)];
            this.G = this.cor(2,1:2) + this.U(2,1:2);
            this.S = [cos(this.phi_aps+this.alpha) -sin(this.phi_aps+this.alpha) ;
                sin(this.phi_aps+this.alpha) cos(this.phi_aps+this.alpha)]*[this.len;0];
            c_kv = this.S+this.G';
            L = norm(c_aps-c_kv);
            deltaL = L-this.L0tr1;
            T = this.k*deltaL;
            dist = c_aps-c_kv;
            n = dist/norm(dist);
            TT = T * n';
            c_aps_sp = [1 * cos(this.phi_aps + this.U(5,3)); 1 * sin(this.phi_aps + this.U(5,3))];
            M_aps = cross([c_aps_sp;0],[TT';0]);
            T_aps = this.F(3,:) + [TT +M_aps(3)];
            % pagreitis
            this.DDU(2,:) = this.pagreitis(T_aps, this.m, this.I_aps); %patikrinti 
            % greitis
            this.DU(2,:) = this.DU(2,:)+dt*this.DDU(2,:);
            % poslinkis
            this.U(2,:) = this.U(2,:)+dt*this.DU(2,:);
            
            return
        end
        
        function DDU = pagreitis(~, F, m, I)
            DDU = F ./ [m m I];
        end
        
        function braizyti_apskritima(this,U,cor,rad)
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi_a=U(3)+cor(3);
            rectangle('Position',[xc-rad,yc-rad,2*rad,2*rad],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]);
            % linija orientacijai pavaizduoti:
            plot([xc,xc+rad*cos(phi_a)], [yc,yc+rad*sin(phi_a)],'k-'); 
        return
        end
        
         function braizyti_trikampi(this,U,cor,rad)
            xc=U(1)+cor(1);yc=U(2)+cor(2);
            fill([this.cor(5,1) this.cor(5,1)-0.5 this.cor(5,1)+0.5], [this.cor(5,2)-0.5 this.cor(5,2)+0.5 this.cor(5,2)+0.5], 1);
            fill([xc xc-0.5 xc+0.5], [yc-0.5 yc+0.5 yc+0.5], rad);
        return
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

