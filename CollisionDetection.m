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
             % apskritimas su kvadratu
             if (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureRectangle'))
                this.circleAgainstRectangle(figure1, figure2);
             elseif (isa(figure2, 'FigureCircle') && isa(figure1, 'FigureRectangle'))
                 this.circleAgainstRectangle(figure2, figure1);
             
             % apskritimas su trikampiu
             elseif (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureTriangle'))
                this.circleAgainstTriangle(figure1, figure2);
             elseif (isa(figure2, 'FigureCircle') && isa(figure1, 'FigureTriangle'))
                 this.circleAgainstTriangle(figure2, figure1);
                 
             % du kvadratai
             elseif (isa(figure1, 'FigureRectangle') && isa(figure2, 'FigureRectangle'))
                 this.RectangleAgainstRectangle(figure1, figure2);
                 this.RectangleAgainstRectangle(figure2, figure1);
             
             % TODO: kvadratas su trikampiu
             % TODO: du trikampiai
             
             % du apskritimai
             elseif (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureCircle'))
                 this.CircleAgainstCircle(figure1, figure2);
             else
                 error('Dar nemoku tokiø kolizijø');
             end
         end
    
         function RectangleAgainstRectangle(this, rectangle1, rectangle2)
            %(this, cor1, a1, b1,U1, direction, cor2, a2, b2, U2)
            %besisukancio kryziaus koordinates
            
            coord1=rectangle1.getCorners();
            kv1_c=rectangle1.getCenter();
            %prijungto
            coord2=rectangle2.getCorners();
            kv2_c=rectangle2.getCenter();
            
            for i=1:4      % ciklas per kli?ties kra?tines 
                j = i+1;
                if j>4, j = 1; end;

                Vi = [coord1(1,i), coord1(2,i)];
                Vj = [coord1(1,j), coord1(2,j)];
                for i2=1:4      % ciklas per kli?ties kra?tines 
                    j2 = i2+1;
                    if j2>4, j2 = 1; end;

                    Vi2 = [coord2(1,i2), coord2(2,i2)];
                    Vj2 = [coord2(1,j2), coord2(2,j2)];
                    
                    [colides, cor] = this.detectLineCollision(Vi, Vj, Vi2, Vj2);
                    if (colides == 1)
                        plot(cor(1),cor(2),'r*','MarkerSize',5)
                        test1=sqrt((cor(1)-Vi(1))^2+(cor(2)-Vi(2))^2);
                        test2=sqrt((cor(1)-Vj(1))^2+(cor(2)-Vj(2))^2);
                        if(test1 < test2)
                            rad = Vi;
                        else
                            rad = Vj;
                        end
                        plot([kv1_c(1),cor(1)], [kv1_c(2),cor(2)],'r-');
                        C_aps=kv1_c;
                        C_kv=kv2_c;
                        K=cor;
                        DU_aps=rectangle1.DU;
                        DU_kv=rectangle2.DU;
                        d = K-C_aps;
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
                        rectangle1.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                        % kvadratà veikianti jëga
                        r = [K-C_kv, 0];
                        mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                        rectangle2.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    end
                end
            end
         end
       function [colides, cor] = detectLineCollision(this, corA1, corA2, corB1, corB2)
           cor = [0, 0];
            %Ax + By = C - tieses lygtis
            %A = y2 - y1
            %B = x1 - x2
            %C = Ax + By
            A1 = corA2(2)-corA1(2);
            A2 = corB2(2)-corB1(2);
            B1 = corA1(1)-corA2(1);
            B2 = corB1(1)-corB2(1);
            C1 = A1 * corA1(1) + B1 * corA1(2);
            C2 = A2 * corB1(1) + B2 * corB1(2);
            %determinantas skaiciuojamas
            det = A1 * B2 - A2 * B1;
            if det == 0
                %lygiagrecios tieses
                colides = 0;
            else
                %tieses kertasi
                %randam x ir y pgl formules
                x = (B2*C1 - B1*C2)/det;
                y = (A1*C2 - A2*C1)/det;
                %tikrinam ar susikirtimo taskas ant abieju figuru krastiniu
                test1 = this.isPointInLine(x, y, corA1, corA2);
                test2 = this.isPointInLine(x, y, corB1, corB2);
                
                %jei taip grazinam colides 0
                if(test1 == 1 && test2 == 1)
%                     plot(x,y,'r*','MarkerSize',5)
                    plot([corA1(1),corA2(1)], [corA1(2),corA2(2)],'r-');
                    plot([corB1(1),corB2(1)], [corB1(2),corB2(2)],'r-');
                    colides = 1;
                    cor = [x, y];
                else
                    colides = 0;
                end
            end
       end 
        %1 if point in line
        %0 if point not in line
        function isPoint = isPointInLine(~,x, y, p1, p2)
            testx = [p1(1) p2(1)]; % x koordinates krastine galu
            testy = [p1(2) p2(2)]; % y koordinates krasitnes galu
            %atrenkam tieses kairiojo ir desiniojo galo koordinates
            if testx(1) > testx(2)
                testx(1) = p2(1);
                testx(2) = p1(1);
            end
            if testy(1) > testy(2)
                testy(1) = p2(2);
                testy(2) = p1(2);
            end

            %jei taskas ant krastines grazinam isPoint 1
            if (x >= testx(1) && x <= testx(2)) && (y >= testy(1) && y <= testy(2))
                isPoint = 1;
            else
                isPoint = 0;
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