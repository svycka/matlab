function spyruokle

% Trikampio inercijos momentas

% Trikamio / daugiakampio transformavimo matrica


g = 9.8;
k = 1000; % spyruokles standumo koef;

% APRASOMAS APSKRITIMAS ---------------------------------------------------
rad = 0.3;
m_aps=1; % mase
I_aps=(m_aps*rad^2)/2;   % inerc. momentas
cor_aps=[-1 0 0];           % staciakampis etalonineje padetyje
F_aps=[0 -m_aps*g 0 ];            % pastovios jegos
U_aps=zeros(1,3);DU_aps=zeros(1,3); DDU_aps=zeros(1,3); 
phi_aps = pi/2; % pradinis posukio kampas;
% END ---------------------------------------------------------------------

% APRASOMAS STACIAKAMPIS --------------------------------------------------
a = 1;  b=0.3;          % staciakampio krastines a ir b
m_kv=2; % mase
I_kv=m_kv*(a^2+b^2)/12;   % inerc. momentas 
cor_kv=[1 0 0];           % staciakampis etalonineje padetyje
F_kv=[0 0 0];            % pastovios jegos
U_kv=zeros(1,3);DU_kv=zeros(1,3); DDU_kv=zeros(1,3); 
% END ---------------------------------------------------------------------


 % Apskritimo mazgas prie kurio jungiama spyruokle ------------------------

c_aps = [cor_aps(1)+rad*cos(phi_aps); cor_aps(2)+rad*sin(phi_aps)];  
% END ---------------------------------------------------------------------

% Kvadrato kampas prie kurio jungiama spyruokle ---------------------------
% Pavyzdyje spyruokle prijungta prie virsutinio kairio kampo

G=U_kv(1:2)+cor_kv(1:2);    % Kvadrato centro kordinates
phi=U_kv(3);                % Kvadrato posukio kampas;

% Pradinis posukio kampas i itvirtinimo mazga
alpha = pi/2 + asin((a/2) / sqrt((a/2)^2 + (b/2)^2));   

% Atkarpos is centro, i itvirtinimo taska ilgis
len = sqrt((a/2)^2 + (b/2)^2);

% Vektorius i� centro i itvirtinimo taska (perkeltas i koord. sistemos pradzia)
S= [cos(phi+alpha) -sin(phi+alpha) ;
         sin(phi+alpha)  cos(phi+alpha)]*[len;0];      
c_kv= S+G';  % Kvadrato kampas prie kurio jungiama spyruokle
% END ---------------------------------------------------------------------

% Spyruokl�s pradinis ilgis -----------------------------------------------
L0 = norm(c_aps-c_kv);   
% END ---------------------------------------------------------------------

xmin=-3; xmax=3; ymin=-4; ymax=4;     % paveikslo ribos


dt = 0.001;
TT = 10;
for t=0:dt:TT
        
    % Apskritimo mazgas prie kurio jungiama spyruokle laiko momentu t -----
    c_aps = [cor_aps(1)+U_aps(1)+rad*cos(phi_aps + U_aps(3)); cor_aps(2)+U_aps(2)+rad*sin(phi_aps +U_aps(3))];  
    % END -----------------------------------------------------------------
    % Vektorisu is centro, i apskritimo taska prie kurio prijungta spyruokle (perkeltas i koordinaciu sistemos pradzia) 
    c_aps_sp = [rad*cos(phi_aps+U_aps(3)); rad*sin(phi_aps+U_aps(3))];  
    % END -----------------------------------------------------------------
    
    
    
    % Kvadrato kampas prie kurio jungiama spyruokle -----------------------
    % alpha, len - kintamieji apskaiciuoti pries sk. integravima
    G=U_kv(1:2)+cor_kv(1:2);    % Kvadrato centro kordinates
    phi=U_kv(3);                % Kvadrato posukio kampas;
    % Vektorius i� centro i itvirtinimo taska
    S= [cos(phi+alpha) -sin(phi+alpha) ;
             sin(phi+alpha)  cos(phi+alpha)]*[len;0];      
    c_kv= S+G';  
    % END -----------------------------------------------------------------
    
    
    % Apskaiciuojamos spyruokles sukurtos jegos ir jegu momentai, ir
    % pridedama prie pastoviu jegu ----------------------------------------
    
    L = norm(c_aps-c_kv);  % Spyruokles ilgis tam tikru laiko momentu
    deltaL = L -L0;        % Spyruokles pokytis nuo pradinio ilgio
    T = k*deltaL;          % Spyruikles sukurta jega
    dist=c_aps-c_kv;  n=dist/norm(dist); % Spyruokles jegos krypties normales vektorius

    TT = T*n'; % Spyruokles sukurta jega, "paskirstyta"x,y kordinatems
    
    M_kv=cross([S;0],[TT'; 0]); % Staciakampio jegos momentas
    M_aps=cross([c_aps_sp;0],[TT'; 0]); % Apskritimo jegos momentas
    
    T_kv = F_kv +[TT +M_kv(3)];    % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
    T_aps = F_aps -[TT +M_aps(3)]; % Prie pastoviu jegu pridedam Spyruokles sukurtas jegas (x,y) asimi ir jegos momenta
    % END -----------------------------------------------------------------
    
    
    % Pagreitis
    DDU_kv=pagreitis(T_kv, m_kv,I_kv); % pagreiciai del isoriniu jegu
    DDU_aps=pagreitis(T_aps, m_aps,I_aps); % pagreiciai del isoriniu jegu
    
    %Greitis    
    DU_kv=DU_kv+dt*DDU_kv;      % greiciu ekstrapoliavimas     
    DU_aps=DU_aps+dt*DDU_aps;      % greiciu ekstrapoliavimas     
    
    % POSLINKIAI
    U_kv=U_kv+dt*DU_kv;
    U_aps=U_aps+dt*DU_aps;
    
    
    % ATVAIZDAVIMAS -------------------------------------------------------
    cla; hold on; figure(1);axis equal;axis([xmin,xmax,ymin,ymax]);hold on, grid on;
    braizyti_staciakampi(U_kv,cor_kv,a, b);
    braizyti_apskritima(U_aps,cor_aps,rad);
    % Nupiesiama spyruokle
    plot([c_aps(1),c_kv(1)], [c_aps(2),c_kv(2)],'r-'); 
    % END -----------------------------------------------------------------
    title(sprintf('t=%g',t));
    pause(dt);
end
end


function DDU = pagreitis(F, m, I)
    DDU = F ./ [m m I];
end

function braizyti_apskritima(U,cor,rad)
    xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
    rectangle('Position',[xc-rad,yc-rad,2*rad,2*rad],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]);
    % linija orientacijai pavaizduoti:
    plot([xc,xc+rad*cos(phi)], [yc,yc+rad*sin(phi)],'k-'); 
return
end

function braizyti_staciakampi(U,cor,a, b)
        xc=U(1)+cor(1);yc=U(2)+cor(2);phi=U(3)+cor(3);
        coord=[-a/2  a/2  a/2 -a/2;
               -b/2 -b/2  b/2  b/2;
                 1    1    1    1  ];   % staciakampis etalonineje padetyje
        T=[cos(phi) -sin(phi) xc;
           sin(phi)  cos(phi) yc;
              0         0      1 ];      % transformavimo matrica
        coord=T*coord;
        fill(coord(1,:),coord(2,:),[0 0.5 0.7]);
    return
end