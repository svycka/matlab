classdef World < handle
    properties(Access=public)
        figures;
        connections;
        collisionDetector;
    end
    
    methods
        function this = World()
            this.collisionDetector = CollisionDetection();
            figure1 = FigureRectangle(1, 1, 2, [1 1.5 3]);
           % figure3 = FigureRectangle(1, 1, 2, [1 3 0]);
%             figure1 = FigureCircle(1, 2, [1.05 2 0]);
%              figure1.DU = [3 0 0];
            figure2 = FigureRectangle(1, 1, 2, [1 0 0]);
            figure2.F = [0 0 0]/10;
            figure2.static = 1;
            this.figures = {figure1 figure2};
           % this.figures = {figure1 figure2 figure3};
            
        end
        
        function move(this, dt)
            for i=1:length(this.connections)
                spring = this.connections{i};
                spring.object1.addForce(spring);
                spring.object2.addForce(spring.inverse());
            end
            
            for i=1:1:length(this.figures)
                for j=i+1:1:length(this.figures)
                    this.collisionDetector.detectCollision(this.figures{i}, this.figures{j});
                end
            end
            
            for i=1:length(this.figures)
                this.figures{i}.move(dt);
            end
        end
        function draw(this)
            % braizom elementus
            for i=1:length(this.figures)
                this.figures{i}.draw();
            end
            
            % braizom spyruokles
            for i=1:length(this.connections)
                this.connections{i}.draw();
            end
        end
    end
    
end

