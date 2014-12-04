classdef CollisionDetection < handle
    properties
        stifp_n=50000;     % baudos standumai(smugio saveikoms)
        dampp_n=50;        % baudos normaliniai klampumai(smugio saveikoms)
        dampp_t=50;        % baudos tangentiniai klampumai(smugio saveikoms)
        fric=0.3;
    end
    
    methods        
         function obj = CollisionDetection()
             
         end
        
         function detectCollision(this, figure1, figure2)
             if (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureRectangle'))
                this.circleAgainstRectangle(figure1, figure2);
             elseif (isa(figure2, 'FigureCircle') && isa(figure1, 'FigureRectangle'))
                 this.circleAgainstRectangle(figure2, figure1);
                 
             elseif (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureTriangle'))
                this.circleAgainstTriangle(figure1, figure2);
             elseif (isa(figure2, 'FigureCircle') && isa(figure1, 'FigureTriangle'))
                 this.circleAgainstTriangle(figure2, figure1);
                 
             elseif (isa(figure1, 'FigureRectangle') && isa(figure2, 'FigureRectangle'))
                 this.RectangleAgainstRectangle(figure1, figure2);
              
             elseif (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureCircle'))
                 this.CircleAgainstCircle(figure1, figure2);
             else
                 error('Dar nemoku tokiø kolizijø');
             end
         end
    
         function RectangleAgainstRectangle(this, rectangle1, rectangle2)
            
            % Kvadratas1
            C_kv1 = rectangle1.getCenter();
            coord_kv1=rectangle1.getCorners();
            DU_kv1 = rectangle1.DU;
            % Kvadratas2
            C_kv2 = rectangle2.getCenter();
            coord_kv2=rectangle2.getCorners();
            DU_kv2 = rectangle2.DU;

            % Kontakto tikrinimas
            for i=1:4      % ciklas per kli?ties kra?tines 
                j = i+1;
                if j>4, j = 1; end;

                Vi = [coord_kv2(1,i), coord_kv2(2,i)];
                Vj = [coord_kv2(1,j), coord_kv2(2,j)];

                Sij = (Vj - Vi) / norm(Vj - Vi);
                
                for ii=1:4
                    jj = ii+1;
                    if jj>4, jj = 1; end;
                    Vi = [coord_kv2(1,i), coord_kv2(2,i)];
                    Vj = [coord_kv2(1,j), coord_kv2(2,j)];

                    Sij = (Vj - Vi) / norm(Vj - Vi);
                    
                    Pi = C_aps-Vi;                    
                    K = Vi + Sij * dot(Sij,Pi);
                    d = K-C_aps;


                    if(norm(d) < rad & dot(Sij,Pi)>=0 & dot(Sij,Pi)<=norm(Vj-Vi)),

                        disp('Apskritimas kontaktuoja su briauna'); 
                    end



                    if norm(Vi - C_aps) < rad,

                        disp('Apskritimas kontaktuoja su kampu'); 
                    end
                end
            end
         end
            
         function CircleAgainstCircle(this, circle1, circle2)
            du=circle1.DU;
            c=circle1.getCenter();
            rad1=circle1.rad;

            duj=circle2.DU;
            cj=circle2.getCenter();
            rad2=circle2.rad;

            n=(cj(1:2)-c(1:2))/norm(cj(1:2)-c(1:2)); tau=[n(2),-n(1)]; %  krypciu vektoriai
            dlt=dot(c(1:2)-cj(1:2),n)+rad1+rad2; % mazgu igilejimas kontakto metu

            if dlt > 0   % ar kunai kontaktuoja
                rN= dlt*this.stifp_n+dot(du(1:2),n)*this.dampp_n; if rN<0, rN=0; end

                rT=(dot(du(1:2)-duj(1:2),tau)-(du(3)*rad1-duj(3)*rad2))*this.dampp_t; 
                if abs(rT)>this.fric*abs(rN), rT=sign(rT)*this.fric*rN; end

                circle1.addDeltaForce(-[rN*n+rT*tau, -rT*rad1]);
                circle2.addDeltaForce(-[ -rN*n-rT*tau,+rT*rad2]);
            end

        end
        
         function circleAgainstRectangle(this, circle, rectangle)
            
            % Apskritimo centras
            C_aps = circle.getCenter();
            rad = circle.rad;
            DU_aps = circle.DU;
            % Kvadratas
            C_kv = rectangle.getCenter();
            coord_kv=rectangle.getCorners();
            DU_kv = rectangle.DU;

            % Kontakto tikrinimas
            for i=1:4      % ciklas per kli?ties kra?tines 
                j = i+1;
                if j>4, j = 1; end;

                Vi = [coord_kv(1,i), coord_kv(2,i)];
                Vj = [coord_kv(1,j), coord_kv(2,j)];

                Sij = (Vj - Vi) / norm(Vj - Vi);
                Pi = C_aps-Vi;                    
                K = Vi + Sij * dot(Sij,Pi);
                d = K-C_aps;


                if(norm(d) < rad & dot(Sij,Pi)>=0 & dot(Sij,Pi)<=norm(Vj-Vi)),
                    dlt = rad - norm(d);

                    % Normalës vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalës vektorius turi bûti statmenas kontakto plokðtumai, o ne iðkeltas ið centro

                    % Jëgos veikianèios apskritimà statmenai normalës vektoriui (statmenos susidûrimo taðkui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jëgos veikianèios apskritimà statmenai tangentës vektoriui (susidûrimo taðko liestinë)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimà veikianti jëga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratà veikianti jëga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    rectangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    disp('Apskritimas kontaktuoja su briauna'); 
                end



                if norm(Vi - C_aps) < rad,
                    % Kontakto taðkas K = virðûnei
                    K = Vi;
                    % Perskaièiuojamas "ágilëjimo atstumas"
                    dlt = rad - norm(Vi - C_aps);

                    % Kaip kontakte su plokðtuma ----------------------------------
                    % Normalës vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalës vektorius turi bûti statmenas kontakto plokðtumai, o ne iðkeltas ið centro

                    % Jëgos veikianèios apskritimà statmenai normalës vektoriui (statmenos susidûrimo taðkui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jëgos veikianèios apskritimà statmenai tangentës vektoriui (susidûrimo taðko liestinë)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimà veikianti jëga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratà veikianti jëga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    rectangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    % Kaip kontakte su kampu  END -----------------------------
                    disp('Apskritimas kontaktuoja su kampu'); 
                end
            end
            
         end
         function circleAgainstTriangle(this, circle, triangle)
            
            % Apskritimo centras
            C_aps = circle.getCenter();
            rad = circle.rad;
            DU_aps = circle.DU;
            % Kvadratas
            C_kv = triangle.getCenter();
            coord_kv=triangle.getCorners();
            DU_kv = triangle.DU;

            % Kontakto tikrinimas
            for i=1:3      % ciklas per kli?ties kra?tines 
                j = i+1;
                if j>3, j = 1; end;

                Vi = [coord_kv(1,i), coord_kv(2,i)];
                Vj = [coord_kv(1,j), coord_kv(2,j)];

                Sij = (Vj - Vi) / norm(Vj - Vi);
                Pi = C_aps-Vi;                    
                K = Vi + Sij * dot(Sij,Pi);
                d = K-C_aps;


                if(norm(d) < rad & dot(Sij,Pi)>=0 & dot(Sij,Pi)<=norm(Vj-Vi)),
                    dlt = rad - norm(d);

                    % Normalës vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalës vektorius turi bûti statmenas kontakto plokðtumai, o ne iðkeltas ið centro

                    % Jëgos veikianèios apskritimà statmenai normalës vektoriui (statmenos susidûrimo taðkui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jëgos veikianèios apskritimà statmenai tangentës vektoriui (susidûrimo taðko liestinë)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimà veikianti jëga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratà veikianti jëga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    triangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    disp('Apskritimas kontaktuoja su briauna'); 
                end



                if norm(Vi - C_aps) < rad,
                    % Kontakto taðkas K = virðûnei
                    K = Vi;
                    % Perskaièiuojamas "ágilëjimo atstumas"
                    dlt = rad - norm(Vi - C_aps);

                    % Kaip kontakte su plokðtuma ----------------------------------
                    % Normalës vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalës vektorius turi bûti statmenas kontakto plokðtumai, o ne iðkeltas ið centro

                    % Jëgos veikianèios apskritimà statmenai normalës vektoriui (statmenos susidûrimo taðkui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jëgos veikianèios apskritimà statmenai tangentës vektoriui (susidûrimo taðko liestinë)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimà veikianti jëga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratà veikianti jëga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    triangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    % Kaip kontakte su kampu  END -----------------------------
                    disp('Apskritimas kontaktuoja su kampu'); 
                end
            end
            
         end
    end
end