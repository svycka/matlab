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
            figure_ver1 = FigureRectangle(1, 16, 1, [-2 0 0]);
            figure_ver2 = FigureRectangle(1, 16, 1, [6 0 0]);
            figure_hor.static = 1;
            figure_ver1.static = 1;
            figure_ver2.static = 1;
            this.addFigure(figure_hor);
            this.addFigure(figure_ver1);
            this.addFigure(figure_ver2);
%             this.addFigure(FigureTriangle(1, 2, [2.1 -1 0]));
%             this.addFigure(FigureTriangle(1, 2, [2 -3 0]));
            
            % add other figures
%             this.testRectangles();
            this.uzduotis();
            
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
        
        function uzduotis(this)
            this.addFigure(FigureRectangle(1, 1, 2, [2 -3 0]));
            this.addFigure(FigureRectangle(1, 1, 2, [2 -2 0]));
            this.addFigure(FigureRectangle(1, 1, 2, [2 -1 0]));
            this.addFigure(FigureRectangle(1, 1, 2, [2 0 0]));
            this.addFigure(FigureRectangle(5, 0.5, 2, [2 0.75 0]));
            this.addFigure(FigureCircle(1, 2, [0 -2.5 0]));
            
        end
        
        function addRandomFigure(this)
            positions = [2,    3.5, 0;
                         -0.5, 3.5, 0;
                         4.5,  3.5, 0];
            randType = randi([1 3],1,1);
            switch randType
                case 1
                    newFigure = FigureCircle(0.5, 2, positions(randi([1 3],1,1),:));
                case 2
                    newFigure = FigureRectangle(1, 1, 2, positions(randi([1 3],1,1),:));
                case 3
                    newFigure = FigureTriangle(1, 2, positions(randi([1 3],1,1),:));
                otherwise
                    error('blogas figuros tipas.')
            end
            
            C1 = newFigure.getCenter();
            for i=1:1:length(this.figures)
                figure = this.figures{i};
                if (figure.static == 1), continue; end;
                minDistance = newFigure.radius + figure.radius;
                C2 = figure.getCenter();
                distance = sqrt((C1(1)-C2(1))^2+(C1(2)-C2(2))^2);
                if distance <= minDistance,
                    return;
                end
            end
                
            newFigure.DU = [randi([-5 5],1,1) randi([-5 5],1,1) randi([-5 5],1,1)];
            this.addFigure(newFigure);
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

