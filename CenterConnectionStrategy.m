classdef CenterConnectionStrategy < ConnectionStrategy
    methods
        function this = CenterConnectionStrategy()
           this.alpha = 0; 
        end
        function point = getConnectionPoint(~, object)
            point = [object.cor(1)+object.U(1); object.cor(2)+object.U(2)];
        end
        
        function vector = getVector(~, ~)
            vector = [0; 0];
        end
    end
    
end

