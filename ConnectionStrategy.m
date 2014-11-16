classdef (Abstract) ConnectionStrategy < handle
    properties
        alpha; % Pradinis posukio kampas i itvirtinimo mazga
        rad = 0; % atstumas iki sujungimo
    end
    
    methods
        function this = ConnectionStrategy()
        end
    end
    methods(Abstract)
        point = getConnectionPoint(this, object)
    end
    
end

