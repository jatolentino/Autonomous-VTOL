clc; clear; close all;

pCs = plantConstants1;
global r initialHeight finalHeight
global xinit


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Calling psi_ref from the trajectoryGen.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1.1 Time constants and loops
Ts = pCs.Ts;                    % Ts=0.1
innerLoops = pCs.innerLoops;    % 4 iterations inside, for the MPC
t =  pCs.t;                     % 0:Ts*innerLoops:100; 
outterLoops = length(t);        % 251 for the Feedback
hz = pCs.hz;                    % 4
ct = pCs.ct;
cq = pCs.cq;
l = pCs.l;

% 2.2 Choosing the trajectory and getting xyz and dot ref
[x_ref,x_dot_ref,x_dotdot_ref,y_ref,y_dot_ref,y_dotdot_ref,z_ref,z_dot_ref,z_dotdot_ref,psi_ref] = trajectoryGen(1,t); % 1 refers to the path1


%% Initial states xt, yt, zt, phit, thetat,psit
xinit = 10;
xt = r ; xinit ;  % t        -7
yt = 0;
zt = initialHeight; % t initialHeight
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

set_omega = 260;
omega1 = set_omega;
omega2 = set_omega;
omega3 = set_omega; 
omega4 = set_omega; 
omega5 = set_omega; 
omega6 = set_omega; 
omega7 = set_omega;
omega8 = set_omega; 
omega9 = set_omega;
omega10 = set_omega;
omega11 = set_omega;
omega12 = set_omega; 
omega_min = 0;  
omega_max = 300;   


% Global sumOmegas
sumOmegas = omega1 - omega2 + omega3 - omega4 + omega5 - omega6 - omega7 + omega8 + omega9 - omega10 - omega11  + omega12; 


% 1.4 Initial thurst THEORY, page 
U1ct512 = pCs.ct*(omega5^2 + omega6^2 + omega7^2 + omega8^2 + omega9^2 + omega10^2 + omega11^2 + omega12^2);
U1 =  pCs.ct*(omega1^2 + omega2^2 + omega3^2 + omega4^2) + U1ct512;

U2ct512 = pCs.ct*pCs.l*(omega5^2*1/2 + omega6^2*sqrt(3)/2 + omega7^2*sqrt(3)/2 + omega8^2*1/2 - omega9^2*1/2 - omega10^2*sqrt(3)/2 - omega11^2*sqrt(3)/2 - omega12^2*1/2);
U2 = pCs.ct*pCs.l*(omega2^2 - omega4^2) + U2ct512;

U3ct512 = pCs.ct*pCs.l*(- omega5^2*sqrt(3)/2 - omega6^2*1/2 + omega7^2*1/2 + omega8^2*sqrt(3)/2 + omega9^2*sqrt(3)/2 + omega10^2*1/2 - omega11^2*1/2 - omega12^2*sqrt(3)/2);
U3 = pCs.ct*pCs.l*(omega3^2 - omega1^2) + U3ct512;

U4ct512 = pCs.cq*(-omega5^2 + omega6^2 + omega7^2 - omega8^2 - omega9^2 - omega10^2 + omega11^2 + omega12^2);
U4 = pCs.cq*(-omega1^2 + omega2^2 - omega3^2 + omega4^2) + U4ct512;

% 2.4 U Inputs maximun and minimun
U_min_sft_f= pCs.U_min_sft_f;    
U_max_sft_f = pCs.U_max_sft_f;

U1_min=(ct*4*omega_min^2 + U1ct512)*U_min_sft_f;
U1_max=(ct*4*omega_max^2 + U1ct512)*U_max_sft_f;

U2_min=(ct*l*(omega_min^2-omega_max^2) + U2ct512)*U_min_sft_f;
U2_max=(ct*l*(omega_max^2-omega_min^2) + U2ct512)*U_max_sft_f;

U3_min=(ct*l*(omega_min^2-omega_max^2) + U3ct512)*U_min_sft_f;
U3_max=(ct*l*(omega_max^2-omega_min^2) + U3ct512)*U_max_sft_f;

U4_min=(cq*(-2*omega_max^2+2*omega_min^2) + U4ct512)*U_min_sft_f;
U4_max=(cq*(-2*omega_min^2+2*omega_max^2) + U4ct512)*U_max_sft_f;

y_max = [U2_max;U3_max;U4_max];
y_min = [U2_min;U3_min;U4_min];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. Calling psi_ref from the trajectoryGen.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2.1 Creating the colum vectors Phi_ref, Theta_ref, Psi_ref
counter = 0;
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
counterx = 0;
ftstotal = {};

%%Input the k1 & k2 TRAINED
% prompt = 'Do you want to use Gradient Descent? Y/N: ';
% stringToComp = input(prompt,'s');
% if isempty(stringToComp )
%     quit cancel;
% end
stringToComp  = "n";

if stringToComp  == "y"
    [k1ex,k2edotx,k1ey,k2edoty,k1ez,k2edotz]=previousTrainingv1();
    kerrorsXYZ = [k1ex,k2edotx,k1ey,k2edoty,k1ez,k2edotz];
    answerGradient = 1;
    disp(kerrorsXYZ)
elseif stringToComp == "n"
    %disp(0);
    answerGradient = 0;
end

for itotal = 1:outterLoops      % total 250
    if answerGradient == 1
        [theta_ref, phi_ref, U1] = positionControlGradient(states,x_ref(itotal),x_dot_ref(itotal),x_dotdot_ref(itotal),y_ref(itotal),...
        y_dot_ref(itotal),y_dotdot_ref(itotal),z_ref(itotal),z_dot_ref(itotal),z_dotdot_ref(itotal),psi_ref(itotal),kerrorsXYZ);
    elseif answerGradient == 0
        [theta_ref, phi_ref, U1] = positionControl(states,x_ref(itotal),x_dot_ref(itotal),x_dotdot_ref(itotal),y_ref(itotal),...
        y_dot_ref(itotal),y_dotdot_ref(itotal),z_ref(itotal),z_dot_ref(itotal),z_dotdot_ref(itotal),psi_ref(itotal));
    end
    counterx = counterx +1;

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
    
    
    % 3.1.1 Making psi increase continuosly in the horizont
    Psi_ref = zeros(1,innerLoops+1)';
    if itotal ~= length(t)
        for itpsi = 1:innerLoops+1   % 4
            Psi_ref(itpsi,1) = psi_ref(itotal) + (itpsi-1)*(psi_ref(itotal+1) - psi_ref(itotal))/innerLoops;
        end
    end
    %disp(Psi_ref(itpsi,1))
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

        
        [Hdb,FdbT,Cdb,Adci,C_cm_g,y_max_global,y_min_global] = attitudeControl(Ad,Bd,Cd,Dd,hz,y_max,y_min);
        counter = counter + 1;
        statesssau{counter}  = statesAugmented;
        Hdbstore{counter} = Hdb;
        FdbTstore{counter} = FdbT;
        Cdbstore{counter} = Cdb;
        Adcistore{counter} = Adci;
        References{counter} = reference;
        
        
        %disp(Hdb)
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
                Utotal = [];
                Utotal = [Utotal;U1,U2,U3,U4];

                U1c = (U1)/(3*ct);
                U2c = (U2)/((sqrt(3)+1)*ct*l);
                U3c = (U3)/((sqrt(3)+1)*ct*l);
                U4c = (U4)/(3*cq);

                omega1part = sqrt(0.25*U1c - 0.5*U3c - 0.25*U4c);
                omega2part = sqrt(0.25*U1c + 0.5*U2c + 0.25*U4c);
                omega3part = sqrt(0.25*U1c + 0.5*U3c - 0.25*U4c);
                omega4part = sqrt(0.25*U1c - 0.5*U2c + 0.25*U4c);

                omega1 = omega1part;
                omega2 = omega2part;
                omega3 = omega3part;
                omega4 = omega4part;

                sumOmegas = omega1 - omega2 + omega3 - omega4 - omega5 + omega6 + omega7 - omega8 - omega9 +omega10 +omega11 -omega12;
                [new_states] = nonlinear_drone_model1(states,U1,U2,U3,U4,sumOmegas);
                states=new_states(end,:);
                totalStates = [totalStates;states];
            else
                disp("irreal");
                disp(itotal);
                break
            end
            
        else
            break
        end
    end
 
    
end


%% Save path

fid = fopen('path.txt','wt');
distancexyzpath = zeros(1,length(t));
for i=2:length(t)
    distancexyzpath(1,i) = sqrt((x_ref(1,i)-x_ref(1,i-1))^2+(y_ref(1,i)-y_ref(1,i-1))^2+(z_ref(1,i)-z_ref(1,i-1))^2);
end
fprintf(fid,'%.8f %.8f %.8f %.8f\n',[x_ref;y_ref;z_ref;distancexyzpath]);
fclose(fid);

xplot = real(totalStates(1:innerLoops:end,7));
yplot = real(totalStates(1:innerLoops:end,8));
zplot = real(totalStates(1:innerLoops:end,9));

ftraj = fopen('traject.txt','wt');
fprintf(ftraj,'%.8f %.8f %.8f\n',[xplot'; yplot'; zplot']);
fclose(ftraj);


% % Plot the trajectory
figure;
subplot(3,1,1)
xPlot(t(1:length(totalStates(1:innerLoops:end,7))),totalStates(1:innerLoops:end,7),'r','LineWidth',1)
hold on
xPlot(t,x_ref,'--b','LineWidth',2)
grid on;
xlim([0 t(length(t))])
ylim([-r+5 r+5])
xlabel('t (s)','FontSize',15,'fontname','times')
ylabel('x-position (m)','FontSize',15,'fontname','times')
legend({'UAV trajectory','Reference path'},'Location','northeast','FontSize',12,'fontname','times')

%figure;
subplot(3,1,2)
set(gca,'fontname','times')
plot(t,y_ref,'--b','LineWidth',2)
hold on
plot(t(1:length(totalStates(1:innerLoops:end,8))),totalStates(1:innerLoops:end,8),'r','LineWidth',1)
grid on;
xlim([0 t(length(t))])
ylim([-r+5 r+5])
xlabel('t (s)','FontSize',15)
ylabel('y-position (m)','FontSize',15)
legend({'Reference Ypath','UAV trajectory'},'Location','northeast','FontSize',12)

subplot(3,1,3)
set(gca,'fontname','times') 
plot(t,z_ref,'--b','LineWidth',2)
hold on
plot(t(1:length(totalStates(1:innerLoops:end,9))),totalStates(1:innerLoops:end,9),'r','LineWidth',1)
grid on;
xlim([0 length(t)])
ylim([0 finalHeight+5])
xlabel('t (s)','FontSize',15)
ylabel('z-position (m)','FontSize',15)
legend({'Reference-Zpath','Drone-trajectory'},'Location','northeast','FontSize',12)


% DRAW THE UAV AND SIMULATE
figure;
%ax1 = axes('XLim',[-r-5 r+5],'YLim',[-r-5 r+5],'ZLim',[0 finalHeight+5]);
ax1 = axes('XLim',[-r-20 r+20],'YLim',[-r-20 r+20],'ZLim',[0 finalHeight+20]);
xticks(-r-20:5:r+20);
yticks(-r-20:5:r+20);
zticks(0:5:finalHeight+20);
%grid minor
hold on;
grid on;
view(3);

xlabel('X(m)') 
ylabel('Y(m)') 
zlabel('Z(m)')

[xa, ya, za] = cylinder([0.2 0.2]);
[xb1,yb1,zb1] = cylinder([0.05 0.05]);
[xb2,yb2,zb2] = cylinder([0.05 0.05]);
[xa2, ya2, za2] = cylinder([0.2 0.2]);

h = thisdron(1.5);

%Create group object and parent surfaces
combined_objects = hgtransform('Parent',ax1);
set(h,'Parent',combined_objects)

% Set the renderer to OpenGL and update the display
set(gcf, 'Render','opengl')
%drawnow
xplot = real(totalStates(1:innerLoops:end,7));
yplot = real(totalStates(1:innerLoops:end,8));
zplot = real(totalStates(1:innerLoops:end,9));

set(gca,'fontname','times') 
plot3(x_ref,y_ref,z_ref,'--b','LineWidth',2)
plot3(xplot,yplot,zplot,'r','LineWidth',1)

legend({'Reference path',strcat("UAV trajectory ",num2str(xinit-xt),'m')},'Location','northeast','FontSize',12,'fontname','times')

trans = makehgtform('translate',[xplot(1) yplot(1) zplot(1)]);
set(combined_objects, 'Matrix', trans);
pause();
for i = 1: length(totalStates(1:innerLoops:end,8))
	trans = makehgtform('translate',[xplot(i) yplot(i) zplot(i)]);
    rotz = makehgtform('zrotate',real(eulerpsi(i)));
    roty = makehgtform('zrotate',real(eulertheta(i)));
    rotx = makehgtform('zrotate',real(eulerphi(i)));
	set(combined_objects, 'Matrix', trans*rotx*roty*rotz);
    if i < 30
        pause(0.05); %0.3
    else
        pause(0.01);
    end
end
%%global axinit
axinit = [10,5,5,-7];
hold on;
mainTestconferenceNov4('y',ax1,axinit,axinit(1),'g',2)
hold on;
mainTestconferenceNov4('n',ax1,axinit,axinit(2),'m',3)
hold on;
mainTestconferenceNov4('y',ax1,axinit,axinit(3),'k',4)
hold on;
mainTestconferenceNov4('y',ax1,axinit,axinit(4),'c',5)