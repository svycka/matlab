classdef Construction 
    properties
        ind = [1 2
               1 3
               1 4
               2 3
               3 2
               4 3
               3 4
               5 3
               6 4
               7 4
               6 7]; %elementai
        cor = [-17 7
                -22 9
                -22 7
                -22 5
                -26 7
                -28 5
                -28 2]; %mazgu koordinates
        IS = [1 1  0 0  0 0  0 0  0 0  0 0  0 0]; %mazgu itvirtinimai
        rad = [-1.5 -1.5
               -0.5 -0.5
               -1.5 -0.5
               -1.5 -0.5
               -0.5 0.5
               -0.5 0.5
               -0.5 0.5
            ]; %nuokrypis
    end
    
    methods
        function create(this)
            %mazgai
            rectangle('Position',[this.cor(1,1)+this.rad(1,1), this.cor(1,2)+this.rad(1,2),3,3],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
            rectangle('Position',[this.cor(2,1)+this.rad(2,1), this.cor(2,2)+this.rad(2,2),1,1],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]); 
            rectangle('Position',[this.cor(3,1)+this.rad(3,1),this.cor(3,2)+this.rad(3,2),3,1]);
            rectangle('Position',[this.cor(4,1)+this.rad(4,1),this.cor(4,2)+this.rad(4,2),3,1]);
%             p1=[this.cor(5,1)   this.cor(5,2)-0.5];
%             p2=[this.cor(5,1)-0.5 this.cor(5,2)+0.5]; 
%             p3=[this.cor(5,1)+0.5 this.cor(5,2)+0.5];
%             x = [p1(1) p2(1) p3(1)]; 
%             y = [p1(2) p2(2) p3(2)]; 
            fill([this.cor(5,1) this.cor(5,1)-0.5 this.cor(5,1)+0.5], [this.cor(5,2)-0.5 this.cor(5,2)+0.5 this.cor(5,2)+0.5], 1);
            fill([this.cor(6,1) this.cor(6,1)-0.5 this.cor(6,1)+0.5], [this.cor(6,2)-0.5 this.cor(6,2)+0.5 this.cor(6,2)+0.5], 1);
            fill([this.cor(7,1) this.cor(7,1)-0.5 this.cor(7,1)+0.5], [this.cor(7,2)-0.5 this.cor(7,2)+0.5 this.cor(7,2)+0.5], 1);
            %elementai
            nme = length(this.ind) %elementu kiekis
            %braizom elementus
            for i=1:nme,
                cxf= this.rad(this.ind(i,1),1);
                cyf= this.rad(this.ind(i,1),2);
                cxt= this.rad(this.ind(i,2),1);
                cyt= this.rad(this.ind(i,2),2);
                
                line([this.cor(this.ind(i,1),1) this.cor(this.ind(i,2),1)],[this.cor(this.ind(i,1),2) this.cor(this.ind(i,2),2)]);
            end
        end
        
    end
    
end

