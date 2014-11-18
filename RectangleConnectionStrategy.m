classdef RectangleConnectionStrategy < ConnectionStrategy
    methods
        function this = RectangleConnectionStrategy(object, position)
            a = object.a;
            b = object.b;
            phi = object.cor(3);
            
            switch position
                case 'right_top'
                    this.alpha = phi+2*pi + asin((b/2) / sqrt((b/2)^2 + (a/2)^2));
                    this.rad = sqrt((a/2)^2+(b/2)^2);
                case 'right_middle'
                    this.alpha = phi+2*pi;
                    this.rad = a/2;
                case 'right_bottom'
                    this.alpha = phi-2*pi - asin((b/2) / sqrt((b/2)^2 + (a/2)^2));
                    this.rad = sqrt((a/2)^2+(b/2)^2);
                case 'middle_top'
                    this.alpha = phi+pi/2;
                    this.rad = b/2;
                case 'middle_bottom'
                    this.alpha = phi-pi/2;
                    this.rad = b/2;
                case 'left_top'
                    this.alpha = phi+pi + asin((b/2) / sqrt((b/2)^2 + (a/2)^2));
                    this.rad = sqrt((a/2)^2+(b/2)^2);
                case 'left_middle'
                    this.alpha = phi+pi;
                    this.rad = a/2;
                case 'left_bottom'
                    this.alpha = phi+ pi + asin((b/2) / sqrt((b/2)^2 + (a/2)^2));
                    this.rad = sqrt((a/2)^2+(b/2)^2);
                otherwise
                    error('bloga pozicija. Dar nemoku tokios suskaiciuot.')
            end
        end
        
        function point = getConnectionPoint(this, object)
            % Kvadrato centro kordinates
            G=object.U(1:2)+object.cor(1:2);
            % Vektorius ið centro i itvirtinimo taska
            S = this.getVector(object);    
            point = S+G'; 
        end
        
        function vector = getVector(this, object)
            phi=object.U(3); % Kvadrato posukio kampas;
            
            % Vektorius ið centro i itvirtinimo taska
            vector= [cos(phi+this.alpha) -sin(phi+this.alpha) ;
                     sin(phi+this.alpha)  cos(phi+this.alpha)]*[this.rad;0];
        end
    end
    
end

