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
%              x0 = xc - width/2;
%              x1 = xc + width/2;
%              y0 = yc + height/2;
%              y1 = yc - height/2;
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
    end
    
end

