classdef RectangleConnectionStrategy < ConnectionStrategy
    methods
        function this = RectangleConnectionStrategy(a, b, position)
            switch position
                % tik kampam
                % this.alpha = alpha + asin((a/2) / sqrt((a/2)^2 + (b/2)^2));
                case 'right_middle'
                    this.alpha = 2*pi;
                    this.rad = a/2;
                case 'left_middle'
                    this.alpha = pi;
                    this.rad = a/2;
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

