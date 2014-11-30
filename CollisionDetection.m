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
            this.circleAgainstRectangle(figure1, figure2);
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
    end
end