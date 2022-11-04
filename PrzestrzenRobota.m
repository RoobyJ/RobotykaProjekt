% P0=[0;0;0;1];
% dlugoscRamienia1 = 870;
% dlugoscRamienia2 = 1016;
% odstepWysokosc = 100;       %odstep kropek po wysokosc toru jezdnego
% odstepSzerokosc = 50;     %odstep kropek po szerokosci/max wychylenie
% odstepKat = 0.08;          %odstep kropek po kacie
% szerokoscRobota = 619;    %szerokosc podstawy robota
% martwePole = 0.476*szerokoscRobota;   %martwe pole przy podstawie robota, obliczone z tg (rys. dlugosc martwego pola)
% torJezdny = 1500;          %dlugosc toru jezdnego
% maxWyprostowanie = 2280;    %dlugosc maksymalnie wyprostowanego robota
% obszarRoboczy = maxWyprostowanie - martwePole;  %obszar ktory osiaga robot
% %   macierze
% figure(1);
% for theta1 = 3.14 : odstepKat : 2*3.14
% for L1 = 0 : odstepSzerokosc : torJezdny
% for L2 = martwePole : obszarRoboczy : maxWyprostowanie
% %   macierze
% A12 = [1 0 0 0; 0 1 0 0; 0 0 1 L1; 0 0 0 1];
% A11 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
%  
% A1=A11*A12;
% A23 = [1 0 0 0; 0 cos(-pi/2) -sin(-pi/2) 0; 0 sin(-pi/2) cos(-pi/2) 0; 0 0 0 1];
% A2=A23;
%  
% A3  = [1 0 0 0; 0 1 0 0; 0 0 1 L2; 0 0 0 1];
% %
% T5=A1*A2*A3;
%  
% P1=T5*P0;
% x=P1(1,1);
% y=P1(2,1);
% z=P1(3,1);
% plot3(x,y,z,'bx'),grid on, hold on;
% end
% end
% end
% x2=0;
% for s1= -maxWyprostowanie : odstepWysokosc : -martwePole
%   %linie proste gora dol
%     for s2= 0 : odstepSzerokosc : torJezdny
%         plot3(x2,s1,s2,'bx'),grid on, hold on;
%         
%     end
% end
%  
% for s3= martwePole : odstepWysokosc + 100 : maxWyprostowanie
%     for s4= 0 : odstepSzerokosc : torJezdny
%         plot3(x2,s3,s4,'bx'),grid on, hold on;
%         title 'Przestrzeń robocza manipulatora';
%     end
% end

katpierwszy =input('kąt obrotu 1 (od -90 do 90 stopni) = '); 
 
katdrugi =input('kąt obrotu 2 (-150 do 150 stopni) = ');
 
wysokosc =input('wysokosc max 1500[mm] = ');
l1 = 870;
l2 = 1016;
katpierwszy =katpierwszy*3.14/180; %radiany
martwePole = 0.476*619;   %martwe pole przy podstawie robota, obliczone z tg (rys. dlugosc martwego pola)
torJezdny = 1500;          %dlugosc toru jezdnego
maxWyprostowanie = 1886;    %dlugosc maksymalnie wyprostowanego robota
katdrugi=katdrugi*3.14/180;

% wzór przedstawiony w sprawozdaniu do wyznaczenia punktu

x1=l1*cos((katpierwszy))+l2*cos((katpierwszy+katdrugi)); y1=l1*sin((katpierwszy))+l2*sin((katpierwszy+katdrugi));
z1=wysokosc; 
r1=sqrt((x1*x1)+(y1*y1));

if (r1<martwePole || r1>maxWyprostowanie || x1<0)
    display('Wartości x i y poza zakresem! Proszę podać inne wartości!');
    x1=input('Punkt początkowy wektora odejścia x: ');
    y1=input('Punkt początkowy wektora odejścia y: ');
end;
r1=sqrt((x1*x1)+(y1*y1));
if (r1<martwePole || r1>maxWyprostowanie || x1<0)
    error('bledne wartosci!');
end;
P0=[0;0;0;1];
%   macierze(przestrzeń robocza)
figure(1);
for theta1 = 3.14 : 0.08 : 2*3.14
for L1 = 0 : 50 : 1500
for L2 = 294 : 1592 : 1886
%   macierze
A12 = [1 0 0 0; 0 1 0 0; 0 0 1 L1; 0 0 0 1];
A11 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
 
A1=A11*A12;
A23 = [1 0 0 0; 0 cos(-pi/2) -sin(-pi/2) 0; 0 sin(-pi/2) cos(-pi/2) 0; 0 0 0 1];
A2=A23;
 
A3  = [1 0 0 0; 0 1 0 0; 0 0 1 L2; 0 0 0 1];
%
T5=A1*A2*A3;
 
P1=T5*P0;
x=P1(1,1);
y=P1(2,1);
z=P1(3,1);
 
plot3(x,y,z,'bd'),hold on;
end
end
end
x2=0;
for s1= -1886 : 50 : -294
    for s2= 0 : 50 : 1500
        plot3(x2,s1,s2,'bd'),hold on;
        
    end
end
 
for s3= 294 : 50 : 1886
    for s4= 0 : 50 : 1500
        plot3(x2,s3,s4,'bd'),hold on;
        
    end
end
 
plot3(x1,y1,z1,'.','MarkerSize',25,'MarkerEdge','r'),
hold on;


