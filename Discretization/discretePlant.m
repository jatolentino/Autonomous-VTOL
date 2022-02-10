function [Ad,Bd,Cd,Dd,xdot,ydot,zdot,phi,theta,psi,phidot,thetadot,psidot] = discretePlant(states,sumOmegas)
    pCs = plantConstants1;
    %sumOmegas = pCs.sumOmegas;
    %global sumOmegas;
    Ix = pCs.Ix;
    Iy = pCs.Iy;
    Iz = pCs.Iz;
    Jtp = pCs.Jtp;
    Ts = pCs.Ts;
    u = states(1);
    v = states(2);
    w = states(3);
    p = states(4);
    q = states(5);
    r = states(6);
    phi = states(10);
    theta =  states(11);
    psi = states(12);

    Rzyx = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(phi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta)         sin(phi)*cos(theta)                             cos(phi)*cos(theta)];
    xdot = [1 0 0]*Rzyx*[u;v;w];
    ydot = [0 1 0]*Rzyx*[u;v;w];
    zdot = [0 0 1]*Rzyx*[u;v;w];
    
    Tmatrix = [1  sin(phi)*tan(theta)   cos(phi)*tan(theta);...
               0  cos(phi)              -sin(phi);...
               0  sin(phi)/cos(theta)   cos(phi)/cos(theta)];
    phidot   = [1 0 0]*Tmatrix*[p;q;r];
    thetadot = [0 1 0]*Tmatrix*[p;q;r];
    psidot   = [0 0 1]*Tmatrix*[p;q;r];
    
    % Computing the matrix A
    A = zeros(6);
    A(1,2) = 1;
    A(2,4) = Jtp*sumOmegas/Ix;
    A(2,6) = (Iy-Iz)*thetadot/Ix;
    A(3,4) = 1;
    A(4,2) = -Jtp*sumOmegas/Iy;
    A(4,6) = (Iz-Ix)*phidot/Iy;
    A(5,6) = 1;
    A(6,2) = (Ix-Iy)*thetadot/(2*Iz);
    A(6,4) = (Ix-Iy)*phidot/(2*Iz);
    
    B = zeros(6,3);
    B(2,1) = 1/Ix;
    B(4,2) = 1/Iy;
    B(6,3) = 1/Iz;
    
    C = zeros(3,6);
    C(1,1) = 1;
    C(2,3) = 1;
    C(3,5) = 1;
    
    D = zeros(3);
    
    % Discretize with Forward Euler method
    Ad = eye(size(A)) + Ts*A;
    Bd = Ts*B;
    Cd = C;
    Dd = D;
end