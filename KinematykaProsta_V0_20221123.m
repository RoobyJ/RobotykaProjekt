
%%

% %Przestrzen robota
P0=[0;0;0;1];
dlugoscRamienia1 = 870;
dlugoscRamienia2 = 1016;
odstepWysokosc = 100;       %odstep kropek po wysokosc toru jezdnego
odstepSzerokosc = 50;     %odstep kropek po szerokosci/max wychylenie
odstepKat = 0.08;          %odstep kropek po kacie
szerokoscRobota = 619;    %szerokosc podstawy robota
martwePole = 0.476*szerokoscRobota;   %martwe pole przy podstawie robota, obliczone z tg (rys. dlugosc martwego pola)
torJezdny = 1500;          %dlugosc toru jezdnego
maxWyprostowanie = 2280;    %dlugosc maksymalnie wyprostowanego robota
obszarRoboczy = maxWyprostowanie - martwePole;  %obszar ktory osiaga robot

%   macierze
figure(1);
for theta1 = 3.14 : odstepKat : 2*3.14
for L1 = 0 : odstepSzerokosc : torJezdny
for L2 = martwePole : obszarRoboczy : maxWyprostowanie

%   macierze
A12 = [1 0 0 0; 0 1 0 0; 0 0 1 L1; 0 0 0 1];
A11 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
 
A1=A11*A12;
A23 = [1 0 0 0; 0 cos(-pi/2) -sin(-pi/2) 0; 0 sin(-pi/2) cos(-pi/2) 0; 0 0 0 1];
A2=A23; 
A3  = [1 0 0 0; 0 1 0 0; 0 0 1 L2; 0 0 0 1];

T5=A1*A2*A3;
 
P1=T5*P0;
x=P1(1,1);
y=P1(2,1);
z=P1(3,1);
plot3(x,y,z,'bx'),grid on, hold on;
end
end
end
x2=0;
for s1= -maxWyprostowanie : odstepWysokosc : -martwePole
    %linie proste gora dol
    for s2= 0 : odstepSzerokosc : torJezdny
        plot3(x2,s1,s2,'bx'),grid on, hold on;    
    end
end
 
for s3= martwePole : odstepWysokosc + 100 : maxWyprostowanie
    for s4= 0 : odstepSzerokosc : torJezdny
        plot3(x2,s3,s4,'bx'),grid on, hold on;
        title 'Przestrzeń robocza manipulatora';
    end
end

%%

% %Kinematyka prosta

n=0;
while (n==0)
 n=0;
 przesuniecie1 = input ('Podaj wartość przesunięcia1 osi przesuwnej z predziału <0;1500>: ');
 teta11 = input ('Podaj wartość kąta obrotu theta1 w drugim złączu z przedzialu <-60;60>: ');
 teta12 = input ('Podaj wartość kąta obrotu theta1 w trzecim złączu z przedziału <-60;60>: ');
 przesuniecie2 = input ('Podaj wartość przesunięcia2 osi przesuwnej z predziału <0;1500>: ');
 teta21 = input ('Podaj wartość kąta obrotu theta2 w drugim złączu z przedzialu <-60;60>: '); 
 teta22 = input ('Podaj wartość kąta obrotu theta2 w trzecim złączu z przedziału <-60;60>: ');
 if ((-60)<=teta11 && teta11<=60 && (-60)<=teta12 && teta12<=60 && 0<=przesuniecie1 && przesuniecie1<=1500 && (-60)<=teta21 && teta21<=60 && (-60)<=teta22 && teta22<=60 && 0<=przesuniecie2 && przesuniecie2<=1500), n=1;
 else 
     disp ('Wartości nie zostały wybrane z zakresu, wprowadź jeszcze raz');
 end
end

przesuniecie1=przesuniecie1/2;
przesuniecie2=przesuniecie2/2;
roznica1=teta21-teta11;
roznica2=teta22-teta12;
roznica3=przesuniecie2-przesuniecie1;
skok1=roznica1/100;
skok2=roznica2/100;
skok3=roznica3/100;
delta_konfiguracyjne=[przesuniecie2-przesuniecie1,teta22-teta12,teta21-teta11];

for i=1:100
 a1=[1,0,0,50; 0,1,0,0; 0,0,1,przesuniecie1; 0,0,0,1];
 b1=[cos(teta11*0.017453293), -sin(teta11*0.017453293),0, dlugoscRamienia1*cos(teta11*0.017453293); sin(teta11*0.017453293), cos(teta11*0.017453293), 0, dlugoscRamienia1*sin(teta11*0.017453293); 0, 0, 1, przesuniecie1; 0,0,0,1];
 c1=[cos(teta12*0.017453293), -sin(teta12*0.017453293),0, dlugoscRamienia2*cos(teta12*0.017453293); sin(teta12*0.017453293), cos(teta12*0.017453293), 0, dlugoscRamienia2*sin(teta12*0.017453293); 0, 0, 1, 0; 0,0,0,1];
 T1=a1*b1*c1;
 plot3(T1(1,4),T1(2,4),T1(3,4), '.r','MarkerSize',15);
 hold on;
 pause(0.01);
 teta11=teta11+skok1;
 teta12=teta12+skok2;
 przesuniecie1=przesuniecie1+skok3;
end

%%

%Wykresy

vmax=[139,188,363];% vtheta 1 dps/vtheta2 dps/ vtheta3 dps

maksymalny_czas_ruchu = abs(delta_konfiguracyjne./vmax);
ramp = 0.1;
v_zlacza=delta_konfiguracyjne./max(maksymalny_czas_ruchu);
time_obl=max(maksymalny_czas_ruchu);
t_accel=ramp*time_obl; %czas rozpedzania

accel=ramp*time_obl; %czas rozpedzania
smax=time_obl; %czas ruchu z pelna predkoscia
t_deccel=smax/1-t_accel; %czas rozpoczecia hamowania
t1=linspace(0,t_accel);
t2=linspace(t_accel*1.01,t_accel+t_deccel); 
t3=linspace((t_accel+t_deccel)*1.01, t_accel*2+t_deccel);
t=[t1 t2 t3];

v1_theta1=(v_zlacza(1)/t_accel)*t1;%predkosci w zlaczu theta1
v2_theta1=(v_zlacza(1)*t2)./t2;
v3_theta1=v_zlacza(1)-(v_zlacza(1)/t_accel)*(t3-t_accel-t_deccel);
v_theta1=[v1_theta1 v2_theta1 v3_theta1];
v1_theta2=(v_zlacza(2)/t_accel)*t1;%predkosci w zlaczu theta2
v2_theta2=(v_zlacza(2)*t2)./t2;
v3_theta2=v_zlacza(2)-(v_zlacza(2)/t_accel)*(t3-t_accel-t_deccel);
v_theta2=[v1_theta2 v2_theta2 v3_theta2];
v1_theta3=(v_zlacza(3)/t_accel)*t1;%predkosci w zlaczu przesuwnym
v2_theta3=(v_zlacza(3)*t2)./t2;
v3_theta3=v_zlacza(3)-(v_zlacza(3)/t_accel)*(t3-t_accel-t_deccel);
v_theta3=[v1_theta3 v2_theta3 v3_theta3];

%%

figure(2)%wykres predkosci zlacz w czasie

plot(t,v_theta1,'r',t,v_theta2,'g',t,v_theta3,'b')
title('Wykres predkosci zlaczy');
xlabel('Czas');
ylabel('Predkosc');
legend('Theta1 [°/s]','Theta2 [°/s]','Theta3 [°/s]');
grid;


% ruch w zlaczu theta1
s1_theta1=0.5*(v_zlacza(1)/t_accel)*t1.*t1;
s1t_accel_theta1=0.5*(v_zlacza(1)/t_accel)*t_accel^2;
s2_theta1=s1t_accel_theta1+v_zlacza(1)*(t2-t_accel);
s2t_deccel_theta1=s1t_accel_theta1+v_zlacza(1)*t_deccel;
s3_theta1=s2t_deccel_theta1+v_zlacza(1)*(t3-t_accel-t_deccel)-0.5*(v_zlacza(1)/t_accel)*(t3-t_accel-t_deccel).*(t3-t_accel-t_deccel);
s3t_accel_theta1=s2t_deccel_theta1+v_zlacza(1)*(t_accel)-0.5*(v_zlacza(1)/t_accel)*t_accel^2;

%ruchu w zlaczu theta2
s1_theta2=0.5*(v_zlacza(2)/t_accel)*t1.*t1;
s1t_accel_theta2=0.5*(v_zlacza(2)/t_accel)*t_accel^2;
s2_theta2=s1t_accel_theta2+v_zlacza(2)*(t2-t_accel);
s2t_deccel_theta2=s1t_accel_theta2+v_zlacza(2)*t_deccel;
s3_theta2=s2t_deccel_theta2+v_zlacza(2)*(t3-t_accel-t_deccel)-0.5*(v_zlacza(2)/t_accel)*(t3-t_accel-t_deccel).*(t3-t_accel-t_deccel);
s3t_accel_theta2=s2t_deccel_theta2+v_zlacza(2)*(t_accel)-0.5*(v_zlacza(2)/t_accel)*t_accel^2;

%ruch w zlaczu theta 3
s1_theta3=0.5*(v_zlacza(3)/t_accel)*t1.*t1;
s1t_accel_theta3=0.5*(v_zlacza(3)/t_accel)*t_accel^2;
s2_theta3=s1t_accel_theta3+v_zlacza(3)*(t2-t_accel);
s2t_deccel_theta3=s1t_accel_theta3+v_zlacza(3)*t_deccel;
s3_theta3=s2t_deccel_theta3+v_zlacza(3)*(t3-t_accel-t_deccel)-0.5*(v_zlacza(3)/t_accel)*(t3-t_accel-t_deccel).*(t3-t_accel-t_deccel);
s3t_accel_theta3=s2t_deccel_theta3+v_zlacza(3)*(t_accel)-0.5*(v_zlacza(3)/t_accel)*t_accel^2;
s_theta1=[s1_theta1 s2_theta1 s3_theta1];%zlacza w czasie t1 t2 t3
s_theta2=[s1_theta2 s2_theta2 s3_theta2];
s_theta3=[s1_theta3 s2_theta3 s3_theta3];

%%

figure(3)

plot(t,s_theta1,'r',t,s_theta2,'g',t,s_theta3,'b')
hold on;
title('Wykresy ruchu w zlaczach');
xlabel('Czas [s]');
ylabel('droga');
legend('Theta1 [°]','Theta2 [°]','Theta3 [°]');
grid;
disp(' ');
disp(['Czas ruchu=' num2str(round(t(300),3)),'s'])

kx=VX./VV; % wspolcznnik v_bezier po X 
ky=VY./VV; % wspolcznnik v_bezier po Y
kz=VZ./VV; % wspolcznnik v_bezier po Z

%%
figure(4)

plot(kx,'r');
hold on;
plot(ky,'g');
hold on;
plot(kz,'b');
legend('kx', 'ky', 'kz');
title({'Wykres wspolczynnikow predkosci'});

%%

figure(5)

czas_calkowity=linspace(0,t3(100));
plot(czas_calkowity,PR1(1,:),'r');
hold on;
plot(czas_calkowity,PR1(2,:),'g');
hold on
plot(czas_calkowity,PR1(3,:),'b');
hold on
legend('Sx','Sy','Sz');
title({'Wykres połozen koncowki'});

%%

figure(6)

plot(TXYZ,VX,'r')
xlabel('czas [s]');
ylabel('VX [mm/s]');
hold on;
subplot(1,1,1);
plot(TXYZ,VY,'g')
xlabel('czas [s]');
ylabel('VY [mm/s]');
hold on;
subplot(1,1,1);
plot(TXYZ,VZ,'b')
xlabel('czas [s]');
ylabel('VZ [mm/s]');
hold on;
subplot(1,1,1);
plot(TXYZ,VV,'k')
xlabel('czas [s]');
ylabel('Predkosc [mm/s]');
hold on;
title({'Wykres predkosci koncowki'});
legend('Vx','Vy','Vz','Vwypadkowa');
grid on;  
