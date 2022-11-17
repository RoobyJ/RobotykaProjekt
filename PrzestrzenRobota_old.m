


%przestrzen robota
P0=[0;0;0;1];
dlugoscRamienia1 = 870;
dlugoscRamienia2 = 1016;
odstepWysokosc = 100;       %odstep kropek po wysokosc toru jezdnego
odstepSzerokosc = 50;     %odstep kropek po szerokosci/max wychylenie
odstepKat = 0.08;          %odstep kropek po kacie
szerokoscRobota = 619;    %szerokosc podstawy robota
martwePole = 0.476*szerokoscRobota;   %martwe pole przy podstawie robota, obliczone z tg (rys. dlugosc martwego pola)
torJezdny = 3000;          %dlugosc toru jezdnego
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
%
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


%ruchy
%n=0;
%while (n==0)
 %n=0;
 teta11 = input ('podaj wartość kąta obrotu theta1 w pierwszym złączu z przedzialu <-60;60>: ');
 teta12 = input ('podaj wartość kąta obrotu theta1 w drugim złączu z przedziału <-60;60>: ');
 przesuniecie1 = input ('podaj wartość przesunięcia1 z predziału<0;1500>: ');
 teta21 = input ('podaj wartość kąta obrotu theta2 w pierwszym złączu z przedzialu <-60;60>: '); 
 teta22 = input ('podaj wartość kąta obrotu theta2 w drugim złączu z przedziału <-60;60>: ');
 przesuniecie2 = input ('podaj wartość przesunięcia2 z predziału<0;1500>: ');
 %if (-90<=teta11 && teta11<=90 && -150<=teta12 && teta12<1580 && 0<=przesuniecie1 && przesuniecie1<=1500 && -90<=teta21 && teta21<=90 && -150<=teta22 && teta22<=150 && 0<=przesuniecie2 && przesuniecie2<=1500)
% n=1;

 %else disp ('wartości nie zostały wybrane z zakresu, wprowadź jeszcze raz');
 %end
 %end

roznica1=teta21-teta11;
roznica2=teta22-teta12;
roznica3=przesuniecie2-przesuniecie1;
skok1=roznica1/100;
skok2=roznica2/100;
skok3=roznica3/100;
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