% this function creates the contact information for the four touch sensor
% arrays of the Barrett Hand.

% finger
% the finger has a 8x3 sensor array, 15mm wide, about 40mm long%
% (projected). The last 3x3 sensor are on the curve of the finger tip
% each sensor is 5mm wide, the first 5 sensors are 6mm long, the curved 3
% sensors are 3mm long.

% The fingers are aligned with the y-axis, and the coordinate origin is a
% the finger tip.

% the spacing in mm
y_spacing = ([0 1 1.5 2.5 6 6 6 6 6 6]-0.5)/1000;
x = [-5 0 5]/1000;
z = [0 1.5 3.5 5  6  6.5 7 7.5 8 8]/1000;

X = [x' x' x' x' x' x' x' x' x' x'];
Y = cumsum([y_spacing;y_spacing;y_spacing],2);
Z = [z; z; z];

Nx = zeros(3,8);

for i=1:8,
    phi=atan2(Z(1,i+2)-Z(1,i),Y(1,i+2)-Y(1,i))+pi/2;
    Ny(1:3,i)=cos(phi);
    Nz(1:3,i)=sin(phi);
end

X=X(:,2:9);
Y=Y(:,2:9);
Z=Z(:,2:9);


figure(1);
clf;
mesh(X,Y,Z);
hold on;
for i=1:8,
  line([X(1,i) X(1,i)+Nx(1,i)/1000],[Y(1,i) Y(1,i)+Ny(1,i)/1000],[Z(1,i) Z(1,i)+Nz(1,i)/1000]);
end
hold off;
axis('equal');


% the palm
yp = [ 2.5 1.5 .5 -.5 -1.5 -2.5]/100;
xp = [ 1.5  0.5  -0.5 -1.5]/100;
zp = [ 0 0 0 0 0 0]/100;

Xp = [xp' xp' xp' xp' xp' xp' ];
Yp = [yp; yp; yp; yp];
Zp = [zp; zp; zp; zp];

Npx = zeros(4,6);
Npy = zeros(4,6);
Npz = ones(4,6);

figure(2);
clf
mesh(Xp,Yp,Zp);
hold on;
for i=1:6,
  line([Xp(1,i) Xp(1,i)+Npx(1,i)/100],[Yp(1,i) Yp(1,i)+Npy(1,i)/100],[Zp(1,i) Zp(1,i)+Npz(1,i)/100]);
end
hold off;
axis('equal');

% print the contact info
for i=1:3
  for j=1:8,
    disp(sprintf('RF2ND    POINT_CONTACT   1   1  half  not_used  % 6.4f % 6.4f % 6.4f  % 6.4f % 6.4f % 6.4f',...
	    X(i,j),Y(i,j),Z(i,j),Nx(i,j),Ny(i,j),Nz(i,j)));
  end
end
    
for i=1:3
  for j=1:8,
    disp(sprintf('LF2ND    POINT_CONTACT   1   1  half  not_used  % 6.4f % 6.4f % 6.4f  % 6.4f % 6.4f % 6.4f',...
	    X(i,j),Y(i,j),Z(i,j),Nx(i,j),Ny(i,j),Nz(i,j)));
  end
end
    
for i=1:3
  for j=1:8,
    disp(sprintf('MF2ND    POINT_CONTACT   1   1  half  not_used  % 6.4f % 6.4f % 6.4f  % 6.4f % 6.4f % 6.4f',...
	    X(i,j),-Y(i,j),Z(i,j),Nx(i,j),-Ny(i,j),Nz(i,j)));
  end
end
    
for i=1:4
  for j=1:6,
    disp(sprintf('PALM    POINT_CONTACT   1   1  half  not_used  % 6.4f % 6.4f % 6.4f  % 6.4f % 6.4f % 6.4f',...
	    Xp(i,j),Yp(i,j),Zp(i,j),Npx(i,j),Npy(i,j),Npz(i,j)));
  end
end
    

