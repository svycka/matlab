classdef World < handle
    properties(Access=public)
        figures;
        connections;
        collisionDetector;
    end
    
    methods
        function this = World()
            this.collisionDetector = CollisionDetection();
            figure1 = FigureCircle(0.3, 2, [-1 1 0]);
             figure1.DU = [3 0 0];
            figure2 = FigureTriangle(1, 2, [1 0 0]);
            figure2.F = [0 -2*9.8 0]/10;
            this.figures = {figure1 figure2};
            
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

