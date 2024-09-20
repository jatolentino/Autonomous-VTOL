function [k1ex,k2edotx,k1ey,k2edoty,k1ez,k2edotz]=previousTrainingv1(axiniti)

% Recalling the constants with the name pCs
pCs = plantConstants;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Initial conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. Calling psi_ref from the trajectoryGen.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2.1 Time constans and loops
Ts = pCs.Ts;                       % Ts=0.1
innerLoops = pCs.innerLoops;    % 4 iterations inside, for the MPC
%t =  pCs.t;       % duration of the simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:Ts*innerLoops:200;         %% ENHCANDE THE TRAINING WIHT AS MUCH AS POINTS POSSIBLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
outterLoops = length(t);        % 251 for the Feedback
hz = pCs.hz;                    % 4
ct = pCs.ct;
cq = pCs.cq;
l = pCs.l;
% 2.2 Choosing the trajectory and getting xyz and dot ref
[x_ref,x_dot_ref,x_dotdot_ref,y_ref,y_dot_ref,y_dotdot_ref,z_ref,z_dot_ref,z_dotdot_ref,psi_ref] = trajectoryGen(1,t); % 1 refers to the path1


% 2.2.1 Initial position and Euler angles
% xt = 0;
% yt = -1;
% zt = 0;
global r initialHeight %xinit finalHeight
xt = axiniti; %xinit;  % r
yt = 0;
zt = initialHeight;
phit = 0;
thetat = 0;
psit = pi/2;

% 2.2.2 Initial velocities
ut = 0;
vt = 0;
wt= 0;
pt = 0;
qt = 0;
rt = 0;

% 2.2.3 PLANT
statesToControl = pCs.statesToControl;
states = [ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit];
totalStates = states;
nestedInertialVel = [x_dot_ref,y_dot_ref,z_dot_ref];

% 1.3 Initial propellers velocities
omega1 = 90*pi; % [rad/s] @ t = -1s  4500*2*pi/60 = 150
omega2 = 90*pi; % [rad/s] @ t = -1s      150
omega3 = 90*pi; % [rad/s] @ t = -1s      150
omega4 = 90*pi; % [rad/s] @ t = -1s      150

omega5 = 90*pi; % [rad/s] @ t = -1s
omega6 = 90*pi; % [rad/s] @ t = -1s
omega7 = 90*pi; % [rad/s] @ t = -1s
omega8 = 90*pi; % [rad/s] @ t = -1s
omega9 = 90*pi; % [rad/s] @ t = -1s
omega10 = 90*pi; % [rad/s] @ t = -1s
omega11 = 90*pi; % [rad/s] @ t = -1s
omega12 = 90*pi; % [rad/s] @ t = -1s

omega_min = 110*pi/3;
omega_max = 860*pi/3;
%global sumOmegas
sumOmegas = omega1 - omega2 + omega3 - omega4 - omega5 + omega6 + omega7 - omega8 - omega9 +omega10 +omega11 -omega12; 

% 1.4 Initial thurst
U1ct512 = omega5^2 + omega6^2 + omega7^2 + omega8^2 + omega9^2 + omega10^2 + omega11^2 + omega12^2;
U1 =  pCs.ct*(omega1^2 + omega2^2 + omega3^2 + omega4^2 + U1ct512);

U2ct512 = omega5^2*1/2 + omega6^2*sqrt(3)/2 + omega7^2*sqrt(3)/2 + omega8^2*1/2 - omega9^2*1/2 - omega10^2*sqrt(3)/2 - omega11^2*sqrt(3)/2 - omega12^2*1/2;
U2 = pCs.ct*pCs.l*(omega2^2 - omega4^2 + U2ct512);

U3ct512 = - omega5^2*sqrt(3)/2 - omega6^2*1/2 + omega7^2*1/2 + omega8^2*sqrt(3)/2 + omega9^2*sqrt(3)/2 + omega10^2*1/2 - omega11^2*1/2 - omega12^2*sqrt(3)/2;
U3 = pCs.ct*pCs.l*(omega3^2 - omega1^2 + U3ct512);

U4ct512 = omega5^2 - omega6^2 - omega7^2 + omega8^2 + omega9^2 - omega10^2 - omega11^2 + omega12^2;
U4 = pCs.cq*(-omega1^2 + omega2^2 - omega3^2 + omega4^2 + U4ct512);

% 2.4 Initial and future States
U_min_sft_f= pCs.U_min_sft_f;    
U_max_sft_f = pCs.U_max_sft_f;

U1_min=ct*4*omega_min^2*U_min_sft_f;
U1_max=ct*4*omega_max^2*U_max_sft_f;

U2_min=ct*l*(omega_min^2-omega_max^2)*U_min_sft_f;
U2_max=ct*l*(omega_max^2-omega_min^2)*U_max_sft_f;

U3_min=ct*l*(omega_min^2-omega_max^2)*U_min_sft_f;
U3_max=ct*l*(omega_max^2-omega_min^2)*U_max_sft_f;

U4_min=cq*(-2*omega_max^2+2*omega_min^2)*U_min_sft_f;
U4_max=cq*(-2*omega_min^2+2*omega_max^2)*U_max_sft_f;

y_max = [U2_max;U3_max;U4_max];
y_min = [U2_min;U3_min;U4_min];


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Calling psi_ref from the trajectoryGen.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.1 Creating the colum vectors Phi_ref, Theta_ref, Psi_ref
counter = 0;
%Hdbstore = [];
Hdbstore = {};
Cdbstore = {};
AdciStore = {};
FdbTstore = {};
statesssau = {};
References = {};
duss = {};
U2s = {};
U3s = {};
U4s = {};
dus3 = {};
phisss = {};

eulerphi(1,1) = phit;
eulertheta(1,1) = thetat;
eulerpsi = psi_ref;

omegas1= {};
omegas2= {};
omegas3= {};
omegas4= {};
duspart = {};

% eulerpsi = [];
counterx = 0;
ftstotal = {};
errorsx = [];
errorsdotx = [];
errorsdotdotx = [];
errorsy = [];
errorsdoty = [];
errorsdotdoty = [];
errorsz = [];
errorsdotz = [];
errorsdotdotz = [];
for itotal = 1:outterLoops      % total 250
    [theta_ref, phi_ref, U1,exdata,exdotdata,exdotdotdata,eydata,eydotdata,eydotdotdata,ezdata,ezdotdata,ezdotdotdata] = ...
    previousPositionControl(states,x_ref(itotal),x_dot_ref(itotal),x_dotdot_ref(itotal),y_ref(itotal),y_dot_ref(itotal),...
    y_dotdot_ref(itotal),z_ref(itotal),z_dot_ref(itotal),z_dotdot_ref(itotal),psi_ref(itotal));
    
    errorsx(itotal,1) = exdata;
    errorsdotx(itotal,1) = exdotdata;
    errorsdotdotx(itotal,1) = exdotdotdata;
    
    errorsy(itotal,1) = eydata;
    errorsdoty(itotal,1) = eydotdata;
    errorsdotdoty(itotal,1) = eydotdotdata;
    
    errorsz(itotal,1) = ezdata;
    errorsdotz(itotal,1) = ezdotdata;
    errorsdotdotz(itotal,1) = ezdotdotdata;
    
    eulerphi(1,itotal) = phi_ref;

    eulertheta(1,itotal) = theta_ref;
    
    Phi_ref = phi_ref*ones(1,innerLoops+1)';
    Theta_ref = theta_ref*ones(1,innerLoops+1)';
    
    if U1<U1_min
        U1=U1_min;
    end
    if U1>U1_max
        U1=U1_max;
    end
    
    % 3.1.1 Making psi increase continuosly in the horizon
    Psi_ref = zeros(1,innerLoops+1)';
    if itotal ~= length(t)
        for itpsi = 1:innerLoops+1   % 4
            Psi_ref(itpsi,1) = psi_ref(itotal) + (itpsi-1)*(psi_ref(itotal+1) - psi_ref(itotal))/innerLoops;
        end
    end
    refNestedAngles = [];
    temporaryAngles = [Phi_ref,Theta_ref,Psi_ref];
    refNestedAngles = [refNestedAngles ; temporaryAngles];
    
    refSignals = zeros(1,length(Phi_ref)*statesToControl);
    kSig = 1;
    for i = 1:statesToControl:length(Phi_ref)*statesToControl       % 1,2,3  4,5,6  [phiref0 thetaref0 psiref0   phiref1 thetaref1 psiref1...
       refSignals(i) = Phi_ref(kSig);
       refSignals(i+1) = Theta_ref(kSig);
       refSignals(i+2) = Psi_ref(kSig);
       kSig = kSig+1;
    end
    
    count = 1;
    hz = pCs.hz;
    for i = 1:innerLoops
        [Ad,Bd,Cd,Dd,xdot,ydot,zdot,phi,theta,psi,phidot,thetadot,psidot] = discretePlant(states,sumOmegas);
        % for graph
        tempInertialVel = [xdot ydot zdot];
        %nestedInertialVel = [];
        nestedInertialVel = [nestedInertialVel;{tempInertialVel}];
        % ----
        statesAugmented = [phi;phidot;theta;thetadot;psi;psidot;U2;U3;U4];
        count = count + statesToControl;
        if count + statesToControl*hz - 1 <= length(refSignals)
            reference = refSignals(count:count+statesToControl*hz-1)';
        else
            reference = refSignals(count:length(refSignals))';
            hz = hz - 1;
        end
        %disp(reference)
        %
        %[Hdb,FdbT,Cdb,Adci] = attitudeControl(Ad,Bd,Cd,Dd,hz);
        [Hdb,FdbT,Cdb,Adci,C_cm_g,y_max_global,y_min_global] = attitudeControl(Ad,Bd,Cd,Dd,hz,y_max,y_min);
        counter = counter + 1;
        statesssau{counter}  = statesAugmented;
        Hdbstore{counter} = Hdb;
        FdbTstore{counter} = FdbT;
        Cdbstore{counter} = Cdb;
        Adcistore{counter} = Adci;
        References{counter} = reference;
        

          
        ft = [statesAugmented',reference']*FdbT;
        if ~isnan(Hdb)
            Plimit=Hdb;
            qlimit=ft;
            CClimit=(C_cm_g*Cdb);
            Glimit=[CClimit;-CClimit];
            CAX=C_cm_g*Adci*statesAugmented;
            h1=y_max_global-CAX;
            h2=-y_min_global+CAX;
            hlimit=[h1;h2]';
            
            if isreal(Plimit) && isreal(qlimit) && isreal(Glimit) && isreal(hlimit)
                du=quadprog(Plimit,qlimit,Glimit,hlimit);
                U2 = U2 + du(1);
                U3 = U3 + du(2);
                U4 = U4 + du(3);

                %disp(counter)
                Utotal = [];
                Utotal = [Utotal;U1,U2,U3,U4];
                
                U1c = U1/ct - U1ct512;
                U2c = U2/(ct*l) - U2ct512;
                U3c = U3/(ct*l) - U3ct512;
                U4c = U4/cq - U4ct512;

                omega1part = sqrt(0.25*U1c - 0.5*U3c - 0.25*U4c);
                omega2part = sqrt(0.25*U1c + 0.5*U2c + 0.25*U4c);
                omega3part = sqrt(0.25*U1c + 0.5*U3c - 0.25*U4c);
                omega4part = sqrt(0.25*U1c - 0.5*U2c + 0.25*U4c);

                omega1 = omega1part;
                omega2 = omega2part;
                omega3 = omega3part;
                omega4 = omega4part;

                %phiss{counter} = phi;
                sumOmegas = omega1 - omega2 + omega3 - omega4;
                %disp(states(1));
                %phisss{counter} = states(1);

                [new_states] = nonlinear_drone_model1(states,U1,U2,U3,U4,sumOmegas);
                states=new_states(end,:);
                totalStates = [totalStates;states];
            else
                break
            end
            
        else
            break
        end
    end
end

%x = [1, 23, 43, 72, 87, 56, 98, 33] ;
%% CHANING TO SUM OF ERRORS ex, exd, exdd, ey, eyd, eydd, ezd, ezdd

your_errorsx = [errorsx(1:8) errorsdotx(1:8) errorsdotdotx(1:8)]; %8
dlmwrite('errx.txt',your_errorsx)

your_errorsy = [errorsy(1:8) errorsdoty(1:8) errorsdotdoty(1:8)];
dlmwrite('erry.txt',your_errorsy)

your_errorsz = [errorsz(1:8) errorsdotz(1:8) errorsdotdotz(1:8)];
dlmwrite('errz.txt',your_errorsz)

% your_errorsx = [errorsx(1:8) errorsdotx(1:8) errorsdotdotx(1:8)]; %8
% dlmwrite('errx.txt',your_errorsx)
% 
% your_errorsy = [errorsy(1:8) errorsdoty(1:8) errorsdotdoty(1:8)];
% dlmwrite('erry.txt',your_errorsy)
% 
% your_errorsz = [errorsz(1:8) errorsdotz(1:8) errorsdotdotz(1:8)];
% dlmwrite('errz.txt',your_errorsz)



%% Initilize Gradient Descent Algorythim - Machine learning

%% Load Data
datax = load('errx.txt');  
datay = load('erry.txt');  
dataz = load('errz.txt');

Xi1 = datax(:, 1:2);
yi1 = datax(:, 3);
mi1 = length(yi1);

Xi2 = datay(:, 1:2);
yi2 = datay(:, 3);
mi2 = length(yi2);

Xi3 = dataz(:, 1:2);
yi3 = dataz(:, 3);
mi3 = length(yi3);

[Xi1 mu1 sigma1] = featureNormalize(Xi1);
[Xi2 mu2 sigma2] = featureNormalize(Xi2);
[Xi3 mu2 sigma3] = featureNormalize(Xi3);
% Add intercept term to X
%X = [ones(m, 1) X];


%% Gradient Descent
%fprintf('Running gradient descent ...\n');

% Choose some alpha value
alpha = 0.03;
num_iters = 400;

% Init Theta and Run Gradient Descent 
thetai1 = zeros(2, 1);
thetai2 = zeros(2, 1);
thetai3 = zeros(2, 1);
[thetai1, J_historyx] = gradientDescentMulti(Xi1, yi1, thetai1, alpha, num_iters);
[thetai2, J_historyy] = gradientDescentMulti(Xi2, yi2, thetai2, alpha, num_iters);
[thetai3, J_historyz] = gradientDescentMulti(Xi3, yi3, thetai3, alpha, num_iters);

%%%Plot the convergence graph
% figure;
% plot(1:numel(J_historyx), J_historyx, '-b', 'LineWidth', 2);
% xlabel('Number of iterations');
% ylabel('Cost J');

%%% Display gradient descent's result
fprintf('Theta computed from gradient descent: \n');
fprintf(' %f \n', thetai1);
fprintf('\n');

k1ex = thetai1(1);
k2edotx = thetai1(2);

k1ey = thetai2(1);
k2edoty = thetai2(2);
    
k1ez = thetai3(1);
k2edotz = thetai3(2);

% disp(thetai1);
% disp(thetai2);
% disp(thetai3);


% 
% figure;
% subplot(3,1,1)
% xPlot(t(1:length(totalStates(1:innerLoops:end,7))),totalStates(1:innerLoops:end,7),'r','LineWidth',1)
% hold on
% xPlot(t,x_ref,'--b','LineWidth',2)
% grid on;
% xlabel('t (s)','FontSize',15,'fontname','times')
% ylabel('x-position (m)','FontSize',15,'fontname','times')
% legend({'UAV trajectory','Reference path'},'Location','northeast','FontSize',12,'fontname','times')
% 
% %figure;
% subplot(3,1,2)
% set(gca,'fontname','times')
% plot(t,y_ref,'--b','LineWidth',2)
% hold on
% plot(t(1:length(totalStates(1:innerLoops:end,8))),totalStates(1:innerLoops:end,8),'r','LineWidth',1)
% grid on;
% xlabel('t (s)','FontSize',15)
% ylabel('y-position (m)','FontSize',15)
% legend({'Reference Ypath','UAV trajectory'},'Location','northeast','FontSize',12)
% 
% subplot(3,1,3)
% set(gca,'fontname','times') 
% plot(t,z_ref,'--b','LineWidth',2)
% hold on
% plot(t(1:length(totalStates(1:innerLoops:end,9))),totalStates(1:innerLoops:end,9),'r','LineWidth',1)
% grid on;
% xlabel('t (s)','FontSize',15)
% ylabel('z-position (m)','FontSize',15)
% legend({'Reference-Zpath','Drone-trajectory'},'Location','northeast','FontSize',12)