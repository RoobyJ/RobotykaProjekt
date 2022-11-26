function [x,y,z] = prostakin(fi1,fi2,fi3)
          r2=870;
r3=1016;
r1=1500;

x1=r2.*cos(fi2).*cos(fi1);
y1=r2.*cos(fi2).*sin(fi1);
z1=r2.*sin(fi2);

x2=r3.*cos(fi2+fi3).*cos(fi1);
y2=r3.*cos(fi2+fi3).*sin(fi1);
z2=r3.*sin(fi2+fi3);

x=x1+x2;
y=y1+y2;
z=z1+z2+r1;

end