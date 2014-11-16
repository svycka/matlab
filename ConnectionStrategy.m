classdef (Abstract) ConnectionStrategy < handle
    properties
        alpha; % Pradinis posukio kampas i itvirtinimo mazga
    end
    
    methods
        function this = ConnectionStrategy()
        end
    end
    methods(Abstract)
        getConnectionPoint(this, object)
    end
    
end

