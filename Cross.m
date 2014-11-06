classdef Cross < handle
    properties 
        % staciakampio krastines a ir b
        a = 0; %ilgis
        b = 0; %plotis     
        cor=[0 0 0]; %centro koordinates
        U=zeros(1,3);DU=[0,0,5]; DDU=zeros(1,3); 
        m=1; % mase
        F=[0 0 0];  % pastovios jegos
        I=0; %innercija
        direction = 'left';
        % Pradiniai greiciai;

    end

    methods
        function obj = Cross(a, b, cor, U, m, direction)
            obj.a = a; 
            obj.b = b;
            obj.cor = cor;
            obj.U = U;
            obj.m = m;
            obj.I = m*(a^2+b^2)/12;
            obj.direction = direction;
        end
        function spinCross(obj,dt)
           obj.DDU=(obj.F./[obj.m,obj.m,obj.I]); % pagreiciai del isoriniu jegu
           obj.DU=obj.DU+dt*obj.DDU;      % greiciu ekstrapoliavimas     
           obj.U=obj.U+dt*obj.DU;
           obj.drawCross(obj.U,obj.cor,obj.a,obj.b);   
           return
        end
        
        function drawCross(this,U,cor,a, b)
            this.drawRectangle(a, b, cor, U);
            this.drawRectangle(b,a,cor, U);
        end
        
        function drawRectangle(this, a, b, cor, U)  
            xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
            coord=[-b/2  b/2  b/2 -b/2;
                   -a/2  -a/2  a/2 a/2;
                     1    1    1    1  ];   % staciakampis etalonineje padetyje
            if strcmp(this.direction, 'right')
                T=[sin(phi) -cos(phi) xc;
                    cos(phi)  sin(phi) yc;
                    0         0      1 ];      % transformavimo matrica
            else
                T=[cos(phi) -sin(phi) xc;
                   sin(phi)  cos(phi) yc;
                    0         0      1 ];      % transformavimo matrica
            end
            coord=T*coord;
            fill(coord(1,:),coord(2,:),[0 1 0.7]);
        end
    end
end
