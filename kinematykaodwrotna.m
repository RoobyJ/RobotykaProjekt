
l1 = 870;
l2 = 1016;
katpierwszy =katpierwszy*3.14/180; %radiany
martwePole = 0.476*619;   %martwe pole przy podstawie robota, obliczone z tg (rys. dlugosc martwego pola)
torJezdny = 1500;          %dlugosc toru jezdnego
maxWyprostowanie = 1886;    %dlugosc maksymalnie wyprostowanego robota
katdrugi=katdrugi*3.14/180;

% wzór przedstawiony w sprawozdaniu do wyznaczenia punktu

x1=input('Punkt początkowy wektora odejścia x: ');
y1=input('Punkt początkowy wektora odejścia y: ');
z1 =input('Punkt początkowy wektora odejścia z: ');

x3=input('Punkt koncowy wektora odejścia x: ');
y3=input('Punkt koncowy wektora odejścia y: ');
z3 =input('Punkt koncowy wektora odejścia z: ');

r1=sqrt((x1*x1)+(y1*y1));
if (r1<martwePole || r1>maxWyprostowanie || x1<0)
    error('bledne wartosci!');
end;

r2=sqrt((x2*x2)+(y2*y2));
if (r2<martwePole || r2>maxWyprostowanie || x2<0)
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
plot3(x3,y3,z3,'.','MarkerSize',25,'MarkerEdge','b'),
plot3([x1,x3],[y1,y3],[z1,z3])
hold on;
