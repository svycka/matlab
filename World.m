classdef World < handle
    properties(Access=public)
        figures;
        connections;
        collisionDetector;
    end
    
    methods
        function this = World()
            this.collisionDetector = CollisionDetection();
            
            % add walls
            figure_hor = FigureRectangle(16, 1, 1, [0 -4 0]);
            figure_ver1 = FigureRectangle(1, 8, 1, [-2 0 0]);
            figure_ver2 = FigureRectangle(1, 8, 1, [6 0 0]);
            figure_hor.static = 1;
            figure_ver1.static = 1;
            figure_ver2.static = 1;
            this.addFigure(figure_hor);
            this.addFigure(figure_ver1);
            this.addFigure(figure_ver2);
            this.addFigure(FigureTriangle(1, 2, [2 -2 0]));
            this.addFigure(FigureTriangle(1, 2, [2 -3 0]));
            
            % add other figures
%             this.testRectangles();
            
        end
        function testRectangles(this)
            this.addFigure(FigureRectangle(1, 1, 2, [0 -2.5 32]));
            this.addFigure(FigureRectangle(2, 1, 2, [2 -1.8 0]));
            this.addFigure(FigureRectangle(2, 0.8, 2, [0.3 0 0]));
            this.addFigure(FigureRectangle(1, 1, 2, [3 -3 0]));
            this.addFigure(FigureRectangle(5, 1, 2, [2 3 0]));
%             this.addFigure(FigureRectangle(1, 1, 2, [2 -3 0]));
%             this.addFigure(FigureRectangle(2, 1, 2, [2 -2 0]));
%             this.addFigure(FigureRectangle(2, 0.8, 2, [2 1 0]));
%             this.addFigure(FigureRectangle(1, 1, 2, [2 3 0]));
%             this.addFigure(FigureTriangle(1, 2, [2 3 0]));
%             this.addFigure(FigureRectangle(5, 1, 2, [2 5 0]));
            
        end
        
        function addFigure(this, figure)
            this.figures{length(this.figures)+1}= figure;
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

