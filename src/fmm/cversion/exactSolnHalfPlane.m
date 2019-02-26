N  = 51; % x,y grid number
Nt = 51; % theta grid number

% domain [-L,L]^2
L = 1;

% numerical infinity
numInfty = 1000000;

% create grid
X  = linspace(-L,L,N);
Y  = linspace(-L,L,N);
TH = linspace(0,2*pi,Nt); 

% grid size
[x,y,th] = meshgrid(X,Y,TH);

xr = x; %x-y;  % watch out, commented out transformation 
yr = y; %x+y;  % does not preserve vector length (divide by sqrt(2)?)

rho = 0.3;

ue = zeros(N,N,Nt);

turnTH1 = th;
turnTH2 = mod(th+pi,2*pi);

changeTH1 = min(turnTH1, abs(turnTH1-2*pi));
changeTH2 = min(turnTH2, abs(turnTH2-2*pi));
changeTH = min(changeTH1, changeTH2);

moveHoriz = xr - rho*sin(changeTH);

ue = moveHoriz + rho*changeTH;
ue(xr <= 0) = 0;

ii = 2:N-1; % interior nodes in x,y

figure(1),clf;
FinalValue = ue(ii,ii,:);
isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,0.5);
hold on;
for kk = 1:10
     isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,0.2*kk);
end

%isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,0.4);
%isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,numInfty/2);
xlabel('x')
ylabel('y')
axis([-L,L,-L,L,0,360]);
title(['N = ' num2str(N), ', N_\theta = ',num2str(Nt)]);
axis square;

