function [X_ref,X_dot_ref,X_dotdot_ref,Y_ref,Y_dot_ref,Y_dotdot_ref,Z_ref,Z_dot_ref,Z_dotdot_ref,psiInt] = trajectoryGen(optionTrajectory,t)
%import constants.*;
global r initialHeight finalHeight

%% 1. Calling the paremeters of the PATH %%%%%
%t = 0:Ts*subIt:100;         % t 
f = 0.01; %<0.01,0.06>, r=5    <0.01,0.03> r=10  k1=k2=-4      <0.01,0.06> r=10 k1=k2=-2      <0.01,0.06> r=10  k1=k2=-1
initialHeight = 5;      % Initial point where the path begins
finalHeight = 1000;  %60 215    % Final point wher the path ends
r = 20;  %20
pathAngle = 2*pi*f*t;
zpathLength =  finalHeight - initialHeight;

%% 2. Defining wind conditions
% C_D_u =  1.9;
% C_D_v = 1.6;
% C_D_w = 1.4;
% 
% % 2.1 Drag force across the section area [m^2]
% A_u = 2*1*0.01+0.05^2;
% A_v = 2*1*0.01+0.05^2;
% A_w = 2*2*1*0.01+0.05^2;

%% 3. Selecting the trajectory %%%%
switch optionTrajectory
    case 0
        X_ref = r/5*sin(pathAngle) + t/100;
        Y_ref = t/100 - 1;
        Z_ref =  initialHeight + t*zpathLength./t(length(t));
    case 1
        X_ref = r*cos(pathAngle);  %pathAngle = 2*pi*f*t;
        Y_ref = r*sin(pathAngle);
        Z_ref = initialHeight + t*zpathLength./t(length(t));
        %psi_refp = atan(Y_ref./X_ref);
    case 2
        Y_ref = t;
        X_ref = 2*sin(2*pi*f*t);
        Z_ref = 2*cos(2*pi*f*t);
        %psi_refp = atan(Y_ref./X_ref);
    case 3
        X_ref = r/5*sin(pathAngle) + t/100;
        Y_ref = t/100 - 1;
        Z_ref =  initialHeight + t*zpathLenght./t(length(t));
end
% for the option 1
X_dot_ref = -r*2*pi*f*sin(pathAngle); % -1.5708
Y_dot_ref = r*2*pi*f*cos(pathAngle);  %0
Z_dot_ref = ones(1,length(t))*zpathLength/t(length(t));

X_dotdot_ref = -r*(2*pi*f)^2*cos(pathAngle); %  -0.2467
Y_dotdot_ref = -r*(2*pi*f)^2*sin(pathAngle); %0
Z_dotdot_ref = zeros(1,length(t));

% for the option 0
% X_dot_ref =  r/5*pathAngle.*cos(pathAngle) + 1/100;
% Y_dot_ref =  1/100*ones(length(t));
% Z_dot_ref =  zpathLength./t(length(t)).*ones(length(t));
% 
% X_dotdot_ref = -r/5*pathAngle.^2.*sin(pathAngle);
% Y_dotdot_ref =  zeros(1,length(t));
% Z_dotdot_ref =  zeros(1,length(t));

%%% 3. Computing deltas of distance in x, y and z %%%%
% Compute psi with the dy/dx
dx = X_ref(2:length(X_ref)) - X_ref(1:length(X_ref)-1);
dy = Y_ref(2:length(Y_ref)) - Y_ref(1:length(Y_ref)-1);
dz = Z_ref(2:length(Z_ref)) - Z_ref(1:length(Z_ref)-1);
dx = [dx(1), dx];
dy = [dy(1), dy];
dz = [dz(1), dz];

%%% 4. Creating the yaw angle of reference psiInt %%%%%
psi = zeros(1,length(X_ref));
psiInt = psi;                   % psiInt is psi_ref
%psi(1) = mod(atan2(Y_ref(1),X_ref(1)),2*pi) + pi/2;
psi(1) = atan2(Y_ref(1),X_ref(1)) + pi/2;
psi(2:length(psi)) = atan2(dy(2:length(dy)),dx(2:length(dy)));
dpsi = psi(2:length(psi))-psi(1:length(psi)-1);
psiInt(1) = psi(1);

for i = 2:length(psiInt)
    if dpsi(i-1) < -pi
        psiInt(i) = psiInt(i-1) + dpsi(i-1) + 2*pi;
    elseif dpsi(i-1) > pi
        psiInt(i) = psiInt(i-1) + dpsi(i-1) - 2*pi;
    elseif dpsi(i-1) < pi
        psiInt(i) = psiInt(i-1) + dpsi(i-1);
    end
end

% psiInt = psi_ref










% anx = axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[-5 5]);
% hold on;
% grid on;
% 
% view(3);
% xlabel('X(m)') 
% ylabel('Y(m)') 
% zlabel('Z(m)')
% 
% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %## Drawing the drone body %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [xc, yc, zc] = cylinder([0.1 0.0]);  % for a cone
% [xi, yi, zi] = cylinder([0.2 0.2]);
% 
% haa(1) = surface(xc, zc, -yc, 'FaceColor', 'red');
% haa(2) = surface(zi, yi, 0.5*xi,'FaceColor', 'blue');
% haa(3) = surface(-zi, yi, 0.5*xi, 'FaceColor', 'yellow');
% haa(4) = surface(xi, -1.5*zi, 0.5*yi, 'FaceColor', 'red');
% haa(5) = surface(xc, (1.5*yc)-1.3, zi, 'FaceColor', 'red');
% combined_objects = hgtransform('Parent',anx);
% set(haa,'Parent',combined_objects)
% set(gcf, 'Render','opengl')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% Chossing the trajectory %%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % switch optionTrajectory
% %     case 1
% %         za = 0:0.01:10;
% %         ya = 2*sin(2*pi*za);
% %         xa = 2*cos(2*pi*za);
% %     %case 2
% % end
% plot3(xa,ya,za,'--b','LineWidth',1)
% hold on
% 
% %% Plot the 
% for i = 1: length(xa)
%     trans = makehgtform('translate',[xa(i) ya(i) za(i)]);
%     rotz = 1; %makehgtform('zrotate',beaz(i));
% 	set(combined_objects, 'Matrix', trans*rotz);
% 	pause(0.3);
% end

