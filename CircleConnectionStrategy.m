classdef CircleConnectionStrategy < ConnectionStrategy
    methods
        function this = CircleConnectionStrategy(alpha, rad)
            this.alpha = alpha;
            this.rad = rad;
        end
        
        function point = getConnectionPoint(this, object)
            point = [object.cor(1)+object.U(1)+this.rad*cos(this.alpha+object.U(3)); 
                object.cor(2)+object.U(2)+this.rad*sin(this.alpha+object.U(3))];
        end
        
        function vector = getVector(this, object)
            vector = [this.rad*cos(object.phi+object.U(3)); this.rad*sin(object.phi+object.U(3))];
        end
    end
    
end

