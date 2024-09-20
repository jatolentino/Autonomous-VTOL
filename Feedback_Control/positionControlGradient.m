function [theta_ref, phi_ref, U1] = positionControlGradient(states,x_ref,x_dot_ref,x_dotdot_ref,y_ref,y_dot_ref,...
    y_dotdot_ref,z_ref,z_dot_ref,z_dotdot_ref,psi_ref,kerrorsXYZ)
pCs = plantConstants;
           %1  2  3  4  5  6  7  8  9  10    11     12  
% states = [ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit];
% x_ref,x_dot_ref,x_dotdot_ref,y_ref,y_dot_ref,y_dotdot_ref,z_ref,z_dot_ref,z_dotdot_ref
g = pCs.g;
% px = pCs.px;
% py = pCs.py;
% pz = pCs.pz;
m = pCs.m;
u = states(1);
v = states(2);
w = states(3);

x = states(7);
y = states(8);
z = states(9);

phi = states(10);
theta = states(11);
psi = states(12);


%% 1. Calculating xdot, ydot and zdot through R*[u;v;w]
% Rotation matrix z-y-x
Rzyx = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(phi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
        cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
        -sin(theta)         sin(phi)*cos(theta)                             cos(phi)*cos(theta)];

xdot = [1 0 0]*Rzyx*[u;v;w];
ydot = [0 1 0]*Rzyx*[u;v;w];
zdot = [0 0 1]*Rzyx*[u;v;w];



ex = x_ref - x;
ey = y_ref - y;
ez = z_ref - z;
exdot = x_dot_ref - xdot;
eydot = y_dot_ref - ydot;
ezdot = z_dot_ref - zdot;

% k1 = -4; k2 = -4; %radious 5
% ux = k1*ex + k2*exdot;  % edotdotx
% uy = k1*ey + k2*eydot;  % edotdoty
% uz = k1*ez + k2*ezdot;  % edotdotz
k1x = kerrorsXYZ(1);
k2x = kerrorsXYZ(2);
k1y = kerrorsXYZ(3);
k2y = kerrorsXYZ(4);
k1z = kerrorsXYZ(5);
k2z = kerrorsXYZ(6);


ux = k1x*ex + k2x*exdot;  % edotdotx
uy = k1y*ey + k2y*eydot;  % edotdoty
uz = k1z*ez + k2z*ezdot;  % edotdotz


% Calculation xdd=vx, ydd=vy and zdd=vz
vx = x_dotdot_ref - ux; % vx = xdd
vy = y_dotdot_ref - uy; % 
vz = z_dotdot_ref - uz; % 

a = vx./(vz + g);
b = vy./(vz + g);
c = cos(psi_ref);
d = sin(psi_ref);


%% Computing theta_ref, phi_ref and U1
theta_ref = atan(a.*c + b.*d);

if or(abs(psi_ref)<pi/4,abs(psi_ref)>3*pi/4)
%if abs(psi_ref)<pi/4 || abs(psi_ref)>3*pi/4
    phi_ref = atan(cos(theta_ref).*(tan(theta_ref).*d-b)./c);
else
    phi_ref = atan(cos(theta_ref).*(a-tan(theta_ref).*c)./d);
end
U1 = m*(vz + g)./(cos(phi_ref).*cos(theta_ref));

end