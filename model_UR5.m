function [KinePara,DispPara,PreSetting] = model_UR5(tcp,base)
%% Parameters Kinematics Model

% Vector to Next Frame ref to Frame0
KinePara.kinv = [0,0,89.159/2;
                 0,135.85,89.159/2;
                 425,-119.7,0;
                 392.25,93,0;
                 0,0,-94.65;
                 0,82.3,0;
                 0,0,0]'/1000;

% Rotation Axis ref to Frame0
KinePara.rotdirc = [3,2,2,2,-3,2];

% Last Joint Frame to Tool Flange
KinePara.flange = [-1,0,0;0,0,1;0,1,0]';

% Base Frame
% KinePara.base = R2T(RotXYZ(pi/2,'RotAxis',2),[0;0;1]);
KinePara.base = base;

% Flange to TCP
KinePara.tcp = tcp;

%% Parameters Draw the Robot

% Joint Size (radius of the circle)
DispPara.jsize = [[1.2,1,1]*0.05, [1,0.8,0.8]*0.04];

% Number of Points on the Circle
DispPara.cirpts = 9;

% How to Draw the Links
% 2,3,1 means draw y dirction first, then z then x
DispPara.offorder = [3,2,1;
                     3,2,1;
                     1,2,3;
                     1,2,3;
                     2,3,1;
                     3,2,1;
                     2,3,1];

%%
% Joint Limit 
PreSetting.jrange = [-1;1]*2*pi*ones(1,6);

% 
PreSetting.Home = [0,-90,90,-150,-90,90]/180*pi;

PreSetting.Name = 'UR5';
end