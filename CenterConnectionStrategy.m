classdef CenterConnectionStrategy
    properties
    end
    
    methods
        function point = getConnectionPoint(~, object)
            point = [object.cor(1)+object.U(1); object.cor(2)+object.U(2)];
        end
    end
    
end

