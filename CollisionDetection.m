classdef CollisionDetection
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods        
         function obj = CollisionDetection()
             
         end
        
         function distance = findDistance(~, fromX, fromY, toX, toY)
            aD = abs(fromX - toX);
            bD = abs(fromY - toY);
            distance = sqrt((aD * aD) + (bD * bD));
         end
        
         function detectCircleCollision(this, cx, cy, r, height, width, cor, U)
             %kvadrato centro koord.
             xc = cor(1)+U(1); yc = cor(2)+U(2);
             %kvadrato posūkio kampas
             phi=(U(3)+cor(3))*-1;
             %kvadrato kampai
            x0 = xc + width/2;
            y0 = yc + height/2;
             %unrotate circle center point
             unrotatedCircleX = cos(phi) * (cx - xc) - sin(phi) * (cy - yc) + xc;
             unrotatedCircleY  = sin(phi) * (cx - xc) + cos(phi) * (cy - yc) + yc;
             %find closest point
             if unrotatedCircleX < (x0-width)
                 closestX = x0-width;
             elseif unrotatedCircleX > x0
                 closestX = x0;
             else
                 closestX = unrotatedCircleX;
             end
             
              if unrotatedCircleY < y0-height
                 closestY = y0-height;
              elseif unrotatedCircleY > y0;
                 closestY = y0;
              else
                 closestY = unrotatedCircleY;
              end
              distance = this.findDistance(unrotatedCircleX,unrotatedCircleY,closestX,closestY);
              if(distance < r)
                  'colision'
              end
         end
         
         function detectRectangleCollision(this, cor1, a1, b1, cor2, a2, b2, U2)
             %besisukancio
             pa1 = [cor1(1)-a1/2    cor1(2)+b1/2];
             pb1 = [pa1(1)+a1        pa1(2)];
             pc1 = [pa1(1)           pa1(2)-b1];
             pd1 = [pa1(1)+a1         pa1(2)-b1];
             %prijungto
             xc=U2(1)+cor2(1);yc=U2(2)+cor2(2);phi=U2(3)+cor2(3);
            coord=[-a2/2  a2/2  a2/2 -a2/2;
                   -b2/2 -b2/2  b2/2  b2/2;
                     1    1    1    1  ];   % staciakampis etalonineje padetyje
            T=[cos(phi) -sin(phi) xc;
               sin(phi)  cos(phi) yc;
                  0         0      1 ];      % transformavimo matrica
            coord=T*coord;
            pa2 = [coord(1,1) coord(2,1)];
            pb2 = [coord(1,2) coord(2,2)];
            pc2 = [coord(1,3) coord(2,3)];
            pd2 = [coord(1,4) coord(2,4)];
            colides = zeros(16,1);
            colides(1) = this.detectLineCollision(pa1, pb1, pa2, pb2);
            colides(2) = this.detectLineCollision(pa1, pb1, pa2, pc2);
            colides(3) = this.detectLineCollision(pa1, pb1, pb2, pd2);
            colides(4) = this.detectLineCollision(pa1, pb1, pc2, pd2);
            
            colides(5) = this.detectLineCollision(pa1, pc1, pa2, pb2);
            colides(6) = this.detectLineCollision(pa1, pc1, pa2, pc2);
            colides(7) = this.detectLineCollision(pa1, pc1, pb2, pd2);
            colides(8) = this.detectLineCollision(pa1, pc1, pc2, pd2);
            
            colides(9) = this.detectLineCollision(pc1, pd1, pa2, pb2);
            colides(10) = this.detectLineCollision(pc1, pd1, pa2, pc2);
            colides(11) = this.detectLineCollision(pc1, pd1, pb2, pd2);
            colides(12) = this.detectLineCollision(pc1, pd1, pc2, pd2);
            
            colides(13) = this.detectLineCollision(pb1, pd1, pa2, pb2);
            colides(14) = this.detectLineCollision(pb1, pd1, pa2, pc2);
            colides(15) = this.detectLineCollision(pb1, pd1, pb2, pd2);
            colides(16) = this.detectLineCollision(pb1, pd1, pc2, pd2);
            
            result = 1;
            for j=1:16,
               result = result * colides(j);
            end
            if result == 0
                'staciakampis kertasi'
            end
         end
         
         function colides = detectLineCollision(~, corA1, corA2, corB1, corB2)
             A1 = corA2(2)-corA1(2);
             A2 = corB2(2)-corB1(2);
             B1 = corA1(1)-corA2(1);
             B2 = corB1(1)-corB2(1);
             C1 = A1 * corA1(1) + B1 * corA1(2);
             C2 = A2 * corB1(1) + B2 * corB1(2);
             
             det = A1 * B2 - A2 * B1;
             if det == 0
                 colides = 1;
             else
                x = (B2*C1 - B1*C2)/det;
                y = (A1*C2 - A2*C1)/det;
                if (x > corB1(1) && x < corB2(1)) && (y < corB1(2) && y > corB2(2))
                    colides = 0;
                else
                    colides = 1;
                end
                 
             end
         end
    end
    
end

