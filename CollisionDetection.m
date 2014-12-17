classdef CollisionDetection < handle
    properties
        stifp_n=10000;     % baudos standumai(smugio saveikoms)
        dampp_n=50;        % baudos normaliniai klampumai(smugio saveikoms)
        dampp_t=50;        % baudos tangentiniai klampumai(smugio saveikoms)
        fric=0.6;
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
%                  this.RectangleAgainstRectangle(figure2, figure1);

             % kvadratas su trikampiu
             elseif (isa(figure1, 'FigureTriangle') && isa(figure2, 'FigureRectangle'))
                 this.RectangleAgainstRectangle(figure1, figure2);
             elseif (isa(figure2, 'FigureTriangle') && isa(figure1, 'FigureRectangle'))
                 this.RectangleAgainstRectangle(figure2, figure1);
                 
             % TODO: du trikampiai
             elseif (isa(figure1, 'FigureTriangle') && isa(figure2, 'FigureTriangle'))
                 this.RectangleAgainstRectangle(figure1, figure2);
                 
             % du apskritimai
             elseif (isa(figure1, 'FigureCircle') && isa(figure2, 'FigureCircle'))
                 this.CircleAgainstCircle(figure1, figure2);
             else
                 error('Dar nemoku tokiï¿½ kolizijï¿½');
             end
         end
         
         function RectangleAgainstRectangle(this, rectangle1, rectangle2)
            if (rectangle1.static == rectangle2.static && rectangle1.static == 1), return; end
            coord1=rectangle1.getCorners();
            kv1_c=rectangle1.getCenter();
            %prijungto
            coord2=rectangle2.getCorners();
            kv2_c=rectangle2.getCenter();
            collotion_count =0;
            % Ciklas per kvadrato kampus;
            for i = 1:length(coord1)
                C = [coord1(1,i), coord1(2,i)]; % i1 kvadrato kampas
                for j1=1:length(coord2)      % ciklas per kli?ties kra?tines 
                    j2 = j1+1;
                    if j2>length(coord2), j2 = 1; end;
                    Vi = [coord2(1,j1), coord2(2,j1)]; % i2 kvadrato 1 kampas
                    Vj = [coord2(1,j2), coord2(2,j2)]; % i2 kvadrato 1 kampas
                    Sij = (Vj - Vi) / norm(Vj - Vi); % 
                    Pi = C-Vi;
                    K = Vi + Sij * dot(Sij,Pi);
                    d = K-C;
%                     norm(d)
                    V_norm = [Sij(2), -Sij(1)];
                    dlt = dot(Vi -C,V_norm);
                    GR_len = 0.3; %%%% Reiktø sutvarkyti sità sàlyga %%%%%%
                    if(dlt < 0  & abs(dlt) < GR_len & dot(Sij,Pi)>=0 & dot(Sij,Pi)<=norm(Vj-Vi)),  
                        [colides, cor] = this.detectLineCollision(kv1_c, C, Vi, Vj);
                        if (colides == 0),
                            continue;
                        end
                        collotion_count = collotion_count+1;
                        plot(C(1),C(2),'r*','MarkerSize',5)
                        % Jegos pirmai figurai
                        if rectangle1.static == 0,
                            fff=[0,0,0];
                            DU_kv1=rectangle1.DU; 
                            V_norm1 = [-Sij(2), Sij(1), 0];
                            fn=max(-dlt*this.stifp_n-dot([DU_kv1(1), DU_kv1(2),0],V_norm1)*this.dampp_n,0); % kontakto normaline jega
                            fff(1:2)=fff(1:2)+fn*V_norm1(1:2); 
                            tau=[-V_norm1(2),V_norm1(1),0];               % tangente
                            ft=dot([DU_kv1(1), DU_kv1(2),0],tau)*this.dampp_t;   % kontakto tangentine jega
                            if fn*this.fric < abs(ft), ft=fn*this.fric*sign(ft); end % apribojimas sausos trinties max jegos reiksme
                            fff(1:2)=fff(1:2)-ft*tau(1:2);
                            r=-[kv1_c(1)-K(1),kv1_c(2)-K(2),0];
                            mom=cross(r,fn*V_norm1-ft*tau); fff(3)=fff(3)+mom(3);        % kontakto jegos momentas
                            rectangle1.addDeltaForce(fff);
                        end
                        % Jegos antrai figurai
                        if rectangle2.static == 0,
                            fff=[0,0,0];
                            DU_kv2=rectangle2.DU; 
                            V_norm2 = [Sij(2), -Sij(1), 0];
                            fn=max(-dlt*this.stifp_n-dot([DU_kv2(1), DU_kv2(2),0],V_norm2)*this.dampp_n,0); % kontakto normaline jega
                            fff(1:2)=fff(1:2)+fn*V_norm2(1:2); 
                            tau=[V_norm2(2),-V_norm2(1),0];               % tangente
                            ft=dot([DU_kv2(1), DU_kv2(2),0],tau)*this.dampp_t;   % kontakto tangentine jega
                            if fn*this.fric < abs(ft), ft=fn*this.fric*sign(ft); end % apribojimas sausos trinties max jegos reiksme
                            fff(1:2)=fff(1:2)-ft*tau(1:2);
                            r=-[kv2_c(1)-K(1),kv2_c(2)-K(2),0];
                            mom=cross(r,fn*V_norm2-ft*tau); fff(3)=fff(3)+mom(3);        % kontakto jegos momentas
                            rectangle2.addDeltaForce(fff);
                        end
                        break;
                    end
                end
            end
            if collotion_count >1, return; end
            coord1=rectangle2.getCorners();
            kv1_c=rectangle2.getCenter();
            %prijungto
            coord2=rectangle1.getCorners();
            kv2_c=rectangle1.getCenter();
            % Ciklas per kvadrato kampus;
            for i = 1:length(coord1)
                C = [coord1(1,i), coord1(2,i)]; % i1 kvadrato kampas
                for j1=1:length(coord2)      % ciklas per kli?ties kra?tines 
                    if collotion_count >1, return; end
                    j2 = j1+1;
                    if j2>length(coord2), j2 = 1; end;
                    Vi = [coord2(1,j1), coord2(2,j1)]; % i2 kvadrato 1 kampas
                    Vj = [coord2(1,j2), coord2(2,j2)]; % i2 kvadrato 1 kampas
                    Sij = (Vj - Vi) / norm(Vj - Vi); % 
                    Pi = C-Vi;
                    K = Vi + Sij * dot(Sij,Pi);
                    d = K-C;
%                     norm(d)
                    V_norm = [Sij(2), -Sij(1)];
                    dlt = dot(Vi -C,V_norm);
                    GR_len = 0.3; %%%% Reiktø sutvarkyti sità sàlyga %%%%%%
                    if(dlt < 0  & abs(dlt) < GR_len & dot(Sij,Pi)>=0 & dot(Sij,Pi)<=norm(Vj-Vi)),  
                        [colides, cor] = this.detectLineCollision(kv1_c, C, Vi, Vj);
                        if (colides == 0),
                            continue;
                        end
                        collotion_count = collotion_count+1;
                        plot(C(1),C(2),'r*','MarkerSize',5)
                        % Jegos pirmai figurai
                        if rectangle2.static == 0,
                            fff=[0,0,0];
                            DU_kv1=rectangle2.DU; 
                            V_norm1 = [-Sij(2), Sij(1), 0];
                            fn=max(-dlt*this.stifp_n-dot([DU_kv1(1), DU_kv1(2),0],V_norm1)*this.dampp_n,0); % kontakto normaline jega
                            fff(1:2)=fff(1:2)+fn*V_norm1(1:2); 
                            tau=[-V_norm1(2),V_norm1(1),0];               % tangente
                            ft=dot([DU_kv1(1), DU_kv1(2),0],tau)*this.dampp_t;   % kontakto tangentine jega
                            if fn*this.fric < abs(ft), ft=fn*this.fric*sign(ft); end % apribojimas sausos trinties max jegos reiksme
                            fff(1:2)=fff(1:2)-ft*tau(1:2);
                            r=-[kv1_c(1)-K(1),kv1_c(2)-K(2),0];
                            mom=cross(r,fn*V_norm1-ft*tau); fff(3)=fff(3)+mom(3);        % kontakto jegos momentas
                            rectangle2.addDeltaForce(fff);
                        end
                        % Jegos antrai figurai
                        if rectangle1.static == 0,
                            fff=[0,0,0];
                            DU_kv2=rectangle1.DU; 
                            V_norm2 = [Sij(2), -Sij(1), 0];
                            fn=max(-dlt*this.stifp_n-dot([DU_kv2(1), DU_kv2(2),0],V_norm2)*this.dampp_n,0); % kontakto normaline jega
                            fff(1:2)=fff(1:2)+fn*V_norm2(1:2); 
                            tau=[V_norm2(2),-V_norm2(1),0];               % tangente
                            ft=dot([DU_kv2(1), DU_kv2(2),0],tau)*this.dampp_t;   % kontakto tangentine jega
                            if fn*this.fric < abs(ft), ft=fn*this.fric*sign(ft); end % apribojimas sausos trinties max jegos reiksme
                            fff(1:2)=fff(1:2)-ft*tau(1:2);
                            r=-[kv2_c(1)-K(1),kv2_c(2)-K(2),0];
                            mom=cross(r,fn*V_norm2-ft*tau); fff(3)=fff(3)+mom(3);        % kontakto jegos momentas
                            rectangle1.addDeltaForce(fff);
                        end
                        break;
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

                    % Normalï¿½s vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalï¿½s vektorius turi bï¿½ti statmenas kontakto plokï¿½tumai, o ne iï¿½keltas iï¿½ centro

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai normalï¿½s vektoriui (statmenos susidï¿½rimo taï¿½kui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai tangentï¿½s vektoriui (susidï¿½rimo taï¿½ko liestinï¿½)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimï¿½ veikianti jï¿½ga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratï¿½ veikianti jï¿½ga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    rectangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    disp('Apskritimas kontaktuoja su briauna'); 
                end



                if norm(Vi - C_aps) < rad,
                    % Kontakto taï¿½kas K = virï¿½ï¿½nei
                    K = Vi;
                    % Perskaiï¿½iuojamas "ï¿½gilï¿½jimo atstumas"
                    dlt = rad - norm(Vi - C_aps);

                    % Kaip kontakte su plokï¿½tuma ----------------------------------
                    % Normalï¿½s vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalï¿½s vektorius turi bï¿½ti statmenas kontakto plokï¿½tumai, o ne iï¿½keltas iï¿½ centro

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai normalï¿½s vektoriui (statmenos susidï¿½rimo taï¿½kui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai tangentï¿½s vektoriui (susidï¿½rimo taï¿½ko liestinï¿½)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimï¿½ veikianti jï¿½ga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratï¿½ veikianti jï¿½ga
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

                    % Normalï¿½s vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalï¿½s vektorius turi bï¿½ti statmenas kontakto plokï¿½tumai, o ne iï¿½keltas iï¿½ centro

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai normalï¿½s vektoriui (statmenos susidï¿½rimo taï¿½kui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai tangentï¿½s vektoriui (susidï¿½rimo taï¿½ko liestinï¿½)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimï¿½ veikianti jï¿½ga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratï¿½ veikianti jï¿½ga
                    r = [K-C_kv, 0];
                    mom=cross(r,rN_kv*[n_kv, 0]-rT_kv*[tau_kv, 0]); 
                    triangle.addDeltaForce(-[-rN_kv*n_kv+rT_kv*tau_kv, -mom(3)]);

                    disp('Apskritimas kontaktuoja su briauna'); 
                end



                if norm(Vi - C_aps) < rad,
                    % Kontakto taï¿½kas K = virï¿½ï¿½nei
                    K = Vi;
                    % Perskaiï¿½iuojamas "ï¿½gilï¿½jimo atstumas"
                    dlt = rad - norm(Vi - C_aps);

                    % Kaip kontakte su plokï¿½tuma ----------------------------------
                    % Normalï¿½s vektorius apskritimo ir kvadrato
                    n_aps =  (C_aps - K) / norm (C_aps - K); tau_aps=[n_aps(2),-n_aps(1)];
                    n_kv = -n_aps;tau_kv=[n_kv(2),-n_kv(1)]; % kvadrato normalï¿½s vektorius turi bï¿½ti statmenas kontakto plokï¿½tumai, o ne iï¿½keltas iï¿½ centro

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai normalï¿½s vektoriui (statmenos susidï¿½rimo taï¿½kui)
                    rN_aps  = dlt*this.stifp_n-dot(DU_aps(1:2) - DU_kv(1:2),n_aps)*this.dampp_n; if rN_aps<0, rN_aps=0; end
                    rN_kv   = dlt*this.stifp_n-dot(DU_kv(1:2)- DU_aps(1:2),n_kv)*this.dampp_n;   if rN_kv<0, rN_kv=0; end

                    % Jï¿½gos veikianï¿½ios apskritimï¿½ statmenai tangentï¿½s vektoriui (susidï¿½rimo taï¿½ko liestinï¿½)
                    rT_aps=(dot(DU_aps(1:2)-DU_kv(1:2),tau_aps) +(DU_aps(3)*rad) - DU_kv(3) * norm(C_aps-K))*this.dampp_t;  %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_aps)>this.fric*abs(rN_aps), rT_aps=sign(rT_aps)*this.fric*rN_aps; end % Slydimo trintis

                    rT_kv=(dot(DU_kv(1:2) - DU_aps(1:2),tau_kv) + (DU_kv(3) * norm(C_aps-K) - DU_aps(3)*rad))*this.dampp_t; %slopimas skaiciuojamas atsizvelgiant i kontaktuojanciu tasku greicius (kontakto tasko greitis = centro greitis +  kampinis greitis)
                    if abs(rT_kv)>this.fric*abs(rN_kv), rT_kv=sign(rT_kv)*this.fric*rN_kv; end % Slydimo trintis

                    % apskritimï¿½ veikianti jï¿½ga
                    circle.addDeltaForce(-[-rN_aps*n_aps+rT_aps*tau_aps, rT_aps*rad]);

                    % kvadratï¿½ veikianti jï¿½ga
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