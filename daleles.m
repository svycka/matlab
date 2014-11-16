function daleles
close all; clc;


stifp_n=10000; % baudos standumai(smugio saveikoms)
dampp_n=10;    % baudos normaliniai klampumai(smugio saveikoms)
dampp_t=10;    % baudos tangentiniai klampumai(smugio saveikoms)
fric=0.01;      % slydimo sausosios trinties koeficientai



% Duomenu aprasymas
mass = [1 1 1 1 ]/5; % Mases
rad  = [0.1 0.1 0.1 0.1]*3; % Kamuoliuku spinduliai
ind  = [   % Elementai
    1 3;
    1 2;
    2 3;
    2 4;
%     3 4;
];
cor = [1 2;
       2 3;
       2 1;
       3 2
       ];
g = 9.81/10;

F =  [0 -mass(1)*g    0  -mass(2)*g   0  -mass(3)*g   0  -mass(4)*g  ];
IS = [0 0            0  0           0  0           1  1 ];

%Spyruokliu standumai
stiff =[100  100  100  100 100 ];
damp_abs = [ 1  1  1  1];

nmz = 4; 
U = zeros(nmz*2, 1);
DU = zeros(nmz*2, 1);
DDU = zeros(nmz*2, 1);

% Skaitinis integravimas

TT = 200; % Skaitinis integravimas atliekamas 10 sek;
dt = 0.001; % Skaitinio integravimo zingsnis
nsteps = TT/dt;
t =0;

figure(1); axis equal; axis([0 25 -5 5]); grid on;

for i=1:nsteps
    t = t+dt;
    
    DDU = pagreitis(U, DU, mass, stiff, damp_abs, F, ind, cor, t, rad, stifp_n, dampp_n, dampp_t, fric);
    DU = DU + dt*DDU;
    
    DU(7) =1; 
    DU(8) =cos(t); 
    
    
    U = U + dt*DU;
    if(mod(i,10) ==0),
        cla; hold on; vaizdavimas(U, ind, cor, rad, F, IS,t);
        pause(0.001);
    end
end


end

function DDU=pagreitis(U, DU, mass, stiff, damp_abs, F, ind, cor, t, rad, stifp_n, dampp_n, dampp_t, fric)
    nmz =length(mass); NN = nmz*2; nel = size(ind,1);
    DDU = zeros(4*2, 1);
    T = F; % Pradines jegos
    % ELEMENTU SUKURIAMOS JEGOS
    if(t <1),
        for i = 1:nel % elementu sukurtos jegos
            ri=ind(i,1); si = ind(i,2);
            r = [(ri-1)*2+1   ri*2];
            s = [(si-1)*2+1   si*2];
            cr = cor(ri,:)'+U(r);     cs = cor(si,:)' + U(s);

            l0 = norm(cor(si,:) - cor(ri,:));
            lrs = norm(cr - cs);

            n = (cs - cr)/lrs;
            Trs = stiff(i) * (lrs-l0);
            T(r) = T(r) + Trs*n';
            T(s) = T(s) - Trs*n';
        end
    end

    % KONTAKTO SAVEIKA SU APATINE PLOKSTUMA
    for i=1:nmz   % saveikos su apribojimo plokstuma(t.y."baudos") jegos
        r=[(i-1)*2+1,i*2]; c=cor(i,:)'+U(r); du=DU(r);
        dx = 0-25; dy = -1-1;
        t =[dx dy]/norm([dx dy]);
        n = [t(2) , -t(1)];
        A=[0 -1]; % taskas plokstumoje

        dlt=dot(c'-A,n)-rad(i); % i mases igilejimas i apribojima; 
        if dlt < 0   % ar kunas pasieke apribojimo plokstuma
             force_n=-dlt*stifp_n-dot(du,n)*dampp_n; % normaline baudos jega 
             if force_n < 0 , 
                 force_n=0; force_t=0;  % normaline jega negali buti neigiama              
             else  % slydimo trintis
                % greitis projektuojamas i kontakto plokstuma ir gaunama liestine jega:
                force_t=-dot(du,t)*dampp_t;   

             end
             T(r)=T(r)+force_n*n+force_t*t;  % baudos jegos pridedamos prie jegu vektoriaus
        end 
    end
    
    for i=1:nmz   % mazgu tarpusavio smugiu "baudos" jegos
        r=[(i-1)*2+1,i*2]; ci=cor(i,:)'+U(r); dui=DU(r);
        for j=i+1:nmz
            s=[(j-1)*2+1,j*2]; cj=cor(j,:)'+U(s); duj=DU(s);
            n=(cj-ci)/norm(cj-ci); % normalines saveikos krypties vektorius
            t=[-n(2); n(1)];  % apribojimo plokstumos liestines vektorius
            dlt=norm(cj-ci)-rad(i)-rad(j); % masiu igilejimas kontakto metu 

            if dlt < 0   % ar kunai susiliete
                 force_n=-dlt*stifp_n-dot(duj-dui,n)*dampp_n; % normaline baudos jega 
                 if force_n < 0   force_n=0; force_t=0; % normaline jega negali buti neigiama
                 else  % slydimo trintis
                    % greitis projektuojamas i kontakto plokstuma ir gaunama liestine jega:
                    force_t=-dot(duj-dui,t)*dampp_t;   

                 end
                 T(r)=T(r)-force_n*n'-force_t*t'; % baudos jegos pridedamos prie jegu vektoriaus: 
                 T(s)=T(s)+force_n*n'+force_t*t';
            end
        end
    end
    
    
    for i = 1:nmz % F=m*a  - apskaiciuojamas pagreitis (a)
        r = [(i-1)*2+1   i*2];
        DDU(r) = T(r) / mass(i);
    end
    
end



function vaizdavimas(U,ind,cor,rad,F,IS, t)
%********************************************************
% *****  pagal apskaiciuota poslinkiu vektoriu **********
% *****  pavaizduojama daleliu konstrukcija    **********
%********************************************************
nmz=length(rad);llsk=2;nel=size(ind,1);

xlim=get(gca,'XLim'); ylim=get(gca,'YLim'); % aktyvuoto paveikslo asiu diapazonai
xn=xlim(2)-xlim(1);yn=ylim(2)-ylim(1); %asiu diapazonai
range=min(xn,yn);           % mazesnysis diapazonas jegu mastelio nustatymui
maxForce=max(abs(F));       % didziausia jegu komponente
mast= range/maxForce*0.1;   % mastelio daugiklis jegu vaizdavimui
constrLength=range/17;      % linijos ilgis greicio krastines salygos vaizdavimui

for i=1:nmz
    % pavaizduojamos daleles :
    r=[(i-1)*llsk+1,i*llsk]; c=cor(i,:)'+U(r); % i-os daleles poslinkiai
    rd=rad(i);  % i-os daleles spindulys
    % pavaizduojama apvali dalele:
    rectangle('Position',[c'-rd,2*rd,2*rd],'Curvature',[1,1],'FaceColor',[0.4 0.6 1]);
        
    % pavaizduojamos dalele veikiancios isorines jegos :
    f=F(r)*mast; % i-os daleles jegos vektorius parinktame mastelyje
    x1=c(1);x2=c(1)+f(1);y1=c(2);y2=c(2)+f(2); % rodykles galai
    line([x1,x2],[y1,y2],'Color','red','LineWidth',1);
    varr=[x1-x2;y1-y2]; varr =varr/norm(varr)*range/40; % rodykles smaigalio sukurimas
    alf=pi/6; transf = [cos(alf) sin(alf);-sin(alf) cos(alf)];
    varr1=transf*varr; line([x2, x2+varr1(1)],[y2, y2+varr1(2)],'Color','red','LineWidth',1);
    varr1=transf'*varr;line([x2, x2+varr1(1)],[y2, y2+varr1(2)],'Color','red','LineWidth',1);
                       
    % pavaizduojamos greiciu krastines salygos :
    constr=IS(r); % i-os daleles tvirtinimai
    if constr(1) ~= 0, line(([c(1), c(1)]),([c(2)-constrLength/2, c(2)+constrLength/2]),'Color',[ 0.2 0.2 0.2],'LineWidth',3);end  
    if constr(2) ~= 0, line(([c(1)-constrLength/2, c(1)+constrLength/2]),([c(2), c(2)]),'Color',[ 0.2 0.2 0.2],'LineWidth',3);end  
end

% pavaizduojami elementai (spyruokles+slopintuvai) :
if(t <1),
    for i=1:nel

        ri=ind(i,1);si=ind(i,2);  % elemento mazgu numeriai
        r=[(ri-1)*llsk+1,ri*llsk];s=[(si-1)*llsk+1,si*llsk]; % elemento mazgu laisves laipsniai konstrukcijos vektoriuje
        cr=cor(ri,:)'+U(r); cs=cor(si,:)'+U(s);   % elemento mazgu koordinates
        plot([cr(1),cs(1)] , [cr(2),cs(2)],'b-');     % elementas pavaizduojamas tieses atkarpa

    end
end


plot([0,25], [-1 ,1], 'k', 'LineWidth',2);

    
return
end
