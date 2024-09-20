classdef plantConstants
   properties (Constant)
   % Drone characteristics
   Ix = 672.31 %695.44          % [kg*m^2]  Moment of nertia in Y
   Iy = 695.44 %672.31          % [kg*m^2]  Moment of inertia in X
   Iz = 1288.38 %0.006           % [kg*m^2]  Moment of inertia in Z[kg*m^2]
   m = 264.45 %0.698 %0.698 fails in 2kg, 1.5kg ok          % [kg]      Drone mass
   Jtp = 7.93*0.7^2 %1.302*10^(-6)  % [N*m*s^2] = kg*m^2 Moment of inertia of the propellers
   l = 2.9 %0.171            % [m]       Drone arm length
   % Matrix weights
%    Q = [10 0 0; 0 10 0; 0 0 10]     % Ouput weights
%    S = [10 0 0; 0 10 0; 0 0 10]     % Final horizon weights 
%    R = [10 0 0; 0 10 0; 0 0 10]     % Input weights
   qpruebax1 = 10
   Q = [plantConstants.qpruebax1 0 0; 0 plantConstants.qpruebax1 0; 0 0 plantConstants.qpruebax1]     % Ouput weights
   S = [plantConstants.qpruebax1 0 0; 0 plantConstants.qpruebax1 0; 0 0 plantConstants.qpruebax1]     % Final horizon weights 
   R = [plantConstants.qpruebax1 0 0; 0 plantConstants.qpruebax1 0; 0 0 plantConstants.qpruebax1]     % Input weights
   Ccm = [0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1]

   
   % Mechanic & fluid properties
   ct = 0.016927         % 0.005979            % 6.9471e-06 = 7.6184*10^(-8)*(60/(2*pi))^2    % [N*s^2]
   cq = 0.002395    %8.4635e-04 % 0.002395 % 2.135e-04 %0.0083   % 2.5889e-07 = 2.6839*10^(-9)*(60/(2*pi))^2    % [N*s^2]
   
%    px=[-1+0j -2+0j]
%    py=[-1+0j -2+0j]
%    pz=[-1+0j -2+0j]
   
   % Plant and Data of states
   statesToControl = 3          % for the MPC, phi, theta, psi
   hz = 4                       % horizon period
   innerLoops = 4              % Loops inside the MPC
   Ts = 0.1                     % Sample time                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   t = 0:plantConstants.Ts*plantConstants.innerLoops:800    %25 duration of the simulation
   % t = 0:0.1*4:50                                                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Environment properties
   %rho = 1.225  % [kg/m^3] air density
   g = 9.81            % [m*s^2]   Earth gravity
   U_min_sft_f=1.05    % Makes the minum input values slightly greater
   U_max_sft_f=0.95
   end
   
end