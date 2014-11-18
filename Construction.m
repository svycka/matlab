classdef Construction < handle
    properties(Access=public)
        construction;
        connections;
    end
    
    methods
        function this = Construction()
            figure1 = FigureFixedCircle(1.5, 2, [-17 7 0]);
            figure2 = FigureCircle(1, 2, [-22 9 0]);
            figure3 = FigureRectangle(3, 1, 2, [-22 7 0]);
            figure4 = FigureRectangle(3, 1, 2, [-22 5 0]);
            figure5 = FigureTriangle(1, 2, [-26 7 0]);
            figure6 = FigureTriangle(1, 2, [-26 5 pi/3]);
            figure7 = FigureTriangle(1, 2, [-26 2 0]);
            this.construction = {figure1 figure2 figure3 figure4 figure5 figure6 figure7};
            
            spring1 = Spring(figure1, CircleConnectionStrategy(pi/2,figure1.rad), figure2,CircleConnectionStrategy(0,figure2.rad));
            spring2 = Spring(figure1, CircleConnectionStrategy(pi,figure1.rad), figure3,RectangleConnectionStrategy(figure3, 'right_middle'));
            spring3 = Spring(figure1, CircleConnectionStrategy(3*pi/2,figure1.rad), figure4,RectangleConnectionStrategy(figure4, 'right_middle'));
            spring4 = Spring(figure2, CenterConnectionStrategy, figure3,RectangleConnectionStrategy(figure3, 'right_middle'));
            spring5 = Spring(figure3, RectangleConnectionStrategy(figure3, 'left_middle'), figure5,CenterConnectionStrategy);
            spring6 = Spring(figure4, RectangleConnectionStrategy(figure4, 'left_middle'), figure6,RectangleConnectionStrategy(figure6, 'right_bottom'));
            spring7 = Spring(figure4, RectangleConnectionStrategy(figure4, 'left_middle'), figure7,RectangleConnectionStrategy(figure7, 'right_bottom'));
            spring8 = Spring(figure6, RectangleConnectionStrategy(figure6, 'left_bottom'), figure7,RectangleConnectionStrategy(figure7, 'middle_top'));
            spring9 = Spring(figure3, RectangleConnectionStrategy(figure3, 'left_middle'), figure4,CenterConnectionStrategy);
            spring10 = Spring(figure2, CenterConnectionStrategy, figure3,RectangleConnectionStrategy(figure3, 'left_middle'));
            spring11 = Spring(figure3, RectangleConnectionStrategy(figure3, 'right_middle'), figure4,CenterConnectionStrategy);
            
            this.connections = {spring1 spring2 spring3 spring4 spring5 spring6 spring7 spring8 spring9 spring10 spring11};
        end
        
        function move(this, dt)
            for i=1:length(this.connections)
                spring = this.connections{i};
                spring.object1.addForce(spring);
                spring.object2.addForce(spring.inverse());
            end
            
            for i=1:length(this.construction)
                this.construction{i}.move(dt);
            end
        end
        function draw(this)
            % braizom elementus
            for i=1:length(this.construction)
                this.construction{i}.draw();
            end
            
            % braizom spyruokles
            for i=1:length(this.connections)
                this.connections{i}.draw();
            end
        end
    end
    
end

