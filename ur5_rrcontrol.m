clc;clear;close all

%% 建模：
% 初始状态所有坐标系姿态都和frame0一致，
% flange frame和6轴frame之间的变换由flange确定，
% flange和tcp之间的变换由tcp确定。
% 零位时坐标系之间的平移由p确定

p = [0,0,89.159/2;
     0,135.85,89.159/2;
     425,-119.7,0;
     392.25,93,0;
     0,0,-94.65;
     0,82.3,0;
     0,0,0]'/1000;

rotdirc = [3,2,2,2,-3,2];
flange = [-1,0,0;0,0,1;0,1,0]'; 
tcp = eye(4);
tcp(1:3,4) = [3,0,8]'/100;

%% Set Initial / Goal Angle

qi = [0,-90,90,-150,-90,90]/180*pi; % Initial Joint Angle
qg = ([30,-60,80,-200,-120,45]+10)/180*pi; % Goal Joint Angle
[~,~,~,~,~,~,~,Tg] = fwk(qg,p,rotdirc,flange,tcp); % Goal Pose

%% Set Iteration Parameter

k = 0.1; % Learning Rate 
max_step = ones(1,6)*1/180*pi; % max angle allowed for each axis
maxloop = 10000; % max loop, avoid singularity
errlim = [1/10/1000,0.1/180*pi]; % break loop threshold (accuracy for position and orientation)

q = qi;
qs = NaN(maxloop,6); % record each step for simulation and debug
qs(1,:) = qi;
count = maxloop+1;

for i = 1:maxloop

    %% fwd kinematics and jacobian
    [T01,T02,T03,T04,T05,T06,T0f,T0t] = fwk(q,p,rotdirc,flange,tcp);
    J0 = jacob0(T01,T02,T03,T04,T05,T06,T0t,rotdirc);
    
    %% error term
    dR = Tg(1:3,1:3) * T0t(1:3,1:3)';
    dp = Tg(1:3,4) - T0t(1:3,4);
    [dw,mag_w] = logSO3(dR);
    mag = [norm(dp);mag_w];

    %% Break the loop
    if mag(1)<=errlim(1) && mag(2)<=errlim(2)
        count = i;
        break
    end

    %% Update angle
    dq = k * (pinv(J0) * [dp;dw])';
    dq = bound2range(dq,max_step);
    q = q + dq;

    %% Save q for simulation
    qs(i+1,:) = q;
end

qs = qs(1:count,:);


%% Simulation to Check the Results
% 这些乱七八糟的设置你不用管，只是为了仿真验证刚刚计算出的qs

% UR5 Model
[KinePara,DispPara,PreSetting] = model_UR5(tcp,eye(4));
myRob = SimpleRobotRJ(KinePara,DispPara); % Construct the Robot
myRob.jrange = PreSetting.jrange; % Joint Limit (Optional)
myRob.home = PreSetting.Home; % Set Home Position (Optional)

% Base Frame and Tool Frame Size (Optional)
% If not specify, will not show frames
myRob.frame0Size = 0.12;
myRob.frametSize = 0.05;
myRob.RobName = PreSetting.Name;

% Show Robot with initial pos
myRob.ShowRobot(qi);

% Show tcp trace
myRob.tcptrace = true;
myRob.keeptrace = true;
pause(0.8)

% Move Along Calculated Path
myRob.MoveAbsJ(qs); 

myRob.tcptrace = false;
myRob.keeptrace = false;
myRob.Jog;

%% fwd kinematics
% 方便你看，没用循环，全部硬写

function [T01,T02,T03,T04,T05,T06,T0f,T0t] = fwk(q,p,rotdirc,flange,tcp)

% 由于我建模时候旋转都是eye，实际上T的旋转部分直接用绕q旋转的旋转矩阵代替就行了
% 这里这么写是考虑到你不同的建模方式。
T01 = [eye(3),p(:,1);0,0,0,1] * rot4x4(q(1),rotdirc(1));
T12 = [eye(3),p(:,2);0,0,0,1] * rot4x4(q(2),rotdirc(2));
T23 = [eye(3),p(:,3);0,0,0,1] * rot4x4(q(3),rotdirc(3));
T34 = [eye(3),p(:,4);0,0,0,1] * rot4x4(q(4),rotdirc(4));
T45 = [eye(3),p(:,5);0,0,0,1] * rot4x4(q(5),rotdirc(5));
T56 = [eye(3),p(:,6);0,0,0,1] * rot4x4(q(6),rotdirc(6));
T6f = [flange,p(:,7);0,0,0,1];

T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;
T05 = T01 * T12 * T23 * T34 * T45;
T06 = T01 * T12 * T23 * T34 * T45 * T56;
T0f = T01 * T12 * T23 * T34 * T45 * T56 * T6f;
T0t = T01 * T12 * T23 * T34 * T45 * T56 * T6f * tcp;
end

%% Jacobian

function J0 = jacob0(T01,T02,T03,T04,T05,T06,T0t,rotdirc)
I = eye(3);
dir = sign(rotdirc);
rotdirc = abs(rotdirc);
axisnum = 6;
J0 = NaN(6,axisnum);
P = T0t(1:3,4);

J0(4:6,1) = T01(1:3,1:3) * I(:,rotdirc(1)) * dir(1);
J0(4:6,2) = T02(1:3,1:3) * I(:,rotdirc(2)) * dir(2);
J0(4:6,3) = T03(1:3,1:3) * I(:,rotdirc(3)) * dir(3);
J0(4:6,4) = T04(1:3,1:3) * I(:,rotdirc(4)) * dir(4);
J0(4:6,5) = T05(1:3,1:3) * I(:,rotdirc(5)) * dir(5);
J0(4:6,6) = T06(1:3,1:3) * I(:,rotdirc(6)) * dir(6);

J0(1:3,1) = cross(J0(4:6,1),P-T01(1:3,4));
J0(1:3,2) = cross(J0(4:6,2),P-T02(1:3,4));
J0(1:3,3) = cross(J0(4:6,3),P-T03(1:3,4));
J0(1:3,4) = cross(J0(4:6,4),P-T04(1:3,4));
J0(1:3,5) = cross(J0(4:6,5),P-T05(1:3,4));
J0(1:3,6) = cross(J0(4:6,6),P-T06(1:3,4));
end

%% rotation matrix

function Tr = rot4x4(q,dir)
q = q*sign(dir);
dir = abs(dir);

c=cos(q); s=sin(q);
if dir==1
    Tr = [1,0,0;0,c,-s;0,s,c];
elseif dir==2
    Tr = [c,0,s;0,1,0;-s,0,c];
elseif dir==3
    Tr = [c,-s,0;s,c,0;0,0,1];
end
Tr = [Tr,[0;0;0];0,0,0,1];
end

%% Bound a Vector in a range but keep ratio
function state = bound2range(state,range)
state = state/max([1,max(state./range,[],'all')]);
end

%% Rotation Matrix to Axis Angle
function [w,mag_w] = logSO3(R)
Vee3 = @(R) [R(3,2);R(1,3);R(2,1)];

ct = min(max((trace(R)-1)/2,-1),1);
mag_w = acos(ct);
w = 1/2/sin(mag_w)*Vee3(R-transpose(R));

if abs(mag_w)<1e-6 || ct>1
    w = [0,0,0]';
    mag_w=0;
elseif abs(mag_w-pi)<1e-6
    [~,~,V] = svd(eye(3)-R);
    w = V(:,end)/norm(V(:,end));
end

w = w.*mag_w;
end
