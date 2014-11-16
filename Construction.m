classdef Construction < handle
    properties(Access=public)
        construction;
        connections;
%         ind = [1 2
%                1 3
%                1 4
%                2 3
%                5 3
%                6 4
%                7 4
%                6 7]; %elementai
%         cor = [-17 7 0
%                 -22 9 0
%                 -22 7 0
%                 -22 5 0
%                 -26 7 0
%                 -28 5 0
%                 -28 2 0]; %mazgu koordinates
    end
    
    methods
        function this = Construction()
            figure1 = FigureFixedCircle(1.5, 2, [-17 7 0]);
            figure2 = FigureCircle(1, 2, [-22 9 0]);
            figure3 = FigureRectangle(3, 1, 2, [-22 7 0]);
            figure4 = FigureRectangle(3, 1, 2, [-22 5 0]);
            figure5 = FigureTriangle(1, 2, [-26 7 0]);
            figure6 = FigureTriangle(1, 2, [-26 5 0]);
            figure7 = FigureTriangle(1, 2, [-26 2 0]);
            this.construction = {figure1 figure2 figure3 figure4 figure5 figure6 figure7};
            
            spring1 = Spring(figure1, CenterConnectionStrategy, figure2,CenterConnectionStrategy);
            spring2 = Spring(figure1, CenterConnectionStrategy, figure3,CenterConnectionStrategy);
            spring3 = Spring(figure1, CenterConnectionStrategy, figure4,CenterConnectionStrategy);
            spring4 = Spring(figure2, CenterConnectionStrategy, figure3,CenterConnectionStrategy);
            spring5 = Spring(figure3, CenterConnectionStrategy, figure5,CenterConnectionStrategy);
            spring6 = Spring(figure4, CenterConnectionStrategy, figure6,CenterConnectionStrategy);
            spring7 = Spring(figure4, CenterConnectionStrategy, figure7,CenterConnectionStrategy);
            spring8 = Spring(figure6, CenterConnectionStrategy, figure7,CenterConnectionStrategy);
            spring9 = Spring(figure3, CenterConnectionStrategy, figure4,CenterConnectionStrategy);
            
            this.connections = {spring1 spring2 spring3 spring4 spring5 spring6 spring7 spring8 spring9};
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

