classdef Spring < handle
    properties
        k = 1000;
        length;
        object1;
        object2;
        strategy1;
        strategy2;
    end
    
    methods
        function this = Spring(object1, strategy1, object2, strategy2)
            this.object1 = object1;
            this.object2 = object2;
            this.strategy1 = strategy1;
            this.strategy2 = strategy2;
            this.length = this.getLength();
        end
        
        function len = getLength(this)
            len = norm(this.leftConnectionPoint()-this.rightConnectionPoint());
        end
        
        function point = leftConnectionPoint(this)
            point = this.connectionPoint(this.object1, this.strategy1);
        end
        
        function point = rightConnectionPoint(this)
            point = this.connectionPoint(this.object2, this.strategy2);
        end
        
        function draw(this)
            left = this.leftConnectionPoint();
            right = this.rightConnectionPoint();
            plot([left(1),right(1)], [left(2),right(2)],'r-'); 
        end
        
        function inversed = inverse(this)
            inversed = Spring(this.object2, this.strategy2, this.object1, this.strategy1);
            inversed.length = this.length;
        end
        
        function point = getVector(this)
            point = this.strategy1.getVector(this.object1);
        end
    end
    
    methods (Access=protected)
        function point = connectionPoint(~, object, strategy)
            point = strategy.getConnectionPoint(object);
        end
    end
    
end

