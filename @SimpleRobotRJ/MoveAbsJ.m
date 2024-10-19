function MoveAbsJ(obj,ags,varargin)
% Move Joint

%% Check Angle
inrange = obj.AgInRange(ags,obj.jrange);
if ~all(inrange)
    if size(ags,1)>1
        disp(find(~inrange))
    end
    disp('Check jrange property to get Joint Range')
    error('Angle is Not in Joint Range')
end

%% Input
default_maxstep = ones(1,size(ags,2))/180*pi;
default_maxstep(4:end) = 5/180*pi;
MotionTypes = {'jump','continous'};
validfun = @(x) any(validatestring(x,MotionTypes));

p = inputParser;
addOptional(p,'MotionType','continous',validfun)
addParameter(p,'TimePause',NaN)
addParameter(p,'AngleStep',default_maxstep)
parse(p,varargin{:});
motiontype = p.Results.MotionType;
timepause = p.Results.TimePause;
maxstep = p.Results.AngleStep;

%% Interpolate Angle
if strcmpi(motiontype,'jump')
    ags = ags(end,:);
else
    if size(ags,1) == 1
        ags = agsteps(obj.cjoint,ags,maxstep);
    end
end

%% Update Plot
[robsc,jointc,jposit,toollinkc,tcpxyz,path,T_tcp] = obj.PrepareMotion(ags);

% clear history tcp trace
if ~obj.keeptrace
    obj.allpath = [NaN;NaN;NaN];
end

% add current to history
if obj.keeptrace && obj.tcptrace
    npath = size(obj.allpath,2);
    obj.allpath = [obj.allpath,path];
end

% clear previous path
if ~obj.tcptrace
    obj.simu.plt_path.XData = NaN;
    obj.simu.plt_path.YData = NaN;
    obj.simu.plt_path.ZData = NaN;
end

% clear previous elips
nn = NaN(2,2);
if ~obj.showellip
    obj.simu.plt_elip.XData = nn;
    obj.simu.plt_elip.YData = nn;
    obj.simu.plt_elip.ZData = nn;
end

npos = size(ags,1);
for i = 1:npos

    %% Joint Center
    obj.simu.plt_jcenter.XData = jposit{i}(1,:);
    obj.simu.plt_jcenter.YData = jposit{i}(2,:);
    obj.simu.plt_jcenter.ZData = jposit{i}(3,:);

    %% Joint Circle
    for j = 1:obj.axisnum
        obj.simu.plt_joint(j).XData = jointc{i}(1,:,j);
        obj.simu.plt_joint(j).YData = jointc{i}(2,:,j);
        obj.simu.plt_joint(j).ZData = jointc{i}(3,:,j);
    end

    %% Links
    obj.simu.plt_linkyzx.XData = robsc{i}(1,:);
    obj.simu.plt_linkyzx.YData = robsc{i}(2,:);
    obj.simu.plt_linkyzx.ZData = robsc{i}(3,:);

    %% Tool
    obj.simu.plt_tool.XData = toollinkc{i}(1,:);
    obj.simu.plt_tool.YData = toollinkc{i}(2,:);
    obj.simu.plt_tool.ZData = toollinkc{i}(3,:);

    %% TCP Frame
    for j = 1:3
        obj.simu.plt_tcpf(j).XData = tcpxyz{i}(1,:,j);
        obj.simu.plt_tcpf(j).YData = tcpxyz{i}(2,:,j);
        obj.simu.plt_tcpf(j).ZData = tcpxyz{i}(3,:,j);
    end

    %% Path
    if obj.tcptrace
        if obj.keeptrace
            obj.simu.plt_path.XData = obj.allpath(1,1:npath+i);
            obj.simu.plt_path.YData = obj.allpath(2,1:npath+i);
            obj.simu.plt_path.ZData = obj.allpath(3,1:npath+i);
        else
            obj.simu.plt_path.XData = path(1,1:i);
            obj.simu.plt_path.YData = path(2,1:i);
            obj.simu.plt_path.ZData = path(3,1:i);
        end
    end

    %% Update Angle Value
    if obj.showag
        obj.simu.tit.String = ...
            sprintf(['%7.2f',repmat(' , %7.2f',1,obj.axisnum-1)],ags(i,:)*180/pi);
    end

    %% Ellipse
    if obj.showellip
        [xx,yy,zz] = obj.get_ellipse('scale',obj.elipSize,...
            'n',obj.elipN,'type',obj.elipType);
        obj.simu.plt_elip.XData = xx;
        obj.simu.plt_elip.YData = yy;
        obj.simu.plt_elip.ZData = zz;  
    end
    drawnow

    obj.cjoint = ags(i,:);
    obj.cpose = T_tcp(:,:,i);
    if ~isnan(timepause)
        pause(timepause);
    end
end

% % Update Current Angle Value
% obj.cjoint = ags(end,:);
% obj.simu.tit.String = ...
%     sprintf(['%5.2f',repmat(' , %5.2f',1,obj.axisnum-1)],ags(npos,:)*180/pi);

% Update GUI Value
if ~isempty(obj.guidata)
    if all(isvalid(obj.guidata.bx))
        for i = 1:length(obj.guidata.bx)
            obj.guidata.bx(i).String = num2str(ags(npos,i)*180/pi,'%.2f');
            obj.guidata.sld(i).Value = ags(npos,i)*180/pi;
        end
    end

    if all(isvalid(obj.guidata.posBox))
        obj.guidata.posBox.String = ...
            sprintf('[ %.2f , %.2f , %.2f ]',obj.cpose(1:3,4)*1000);
    end
end
end


%%
function [ags,numags] = agsteps(ag1,ag2,maxstep)

if all(ag1==ag2)
    ags = ag1;
else
%dag = max(abs(ag1-ag2));
%numags = ceil(dag/maxstep);
numags = max(ceil(abs(ag1-ag2)./maxstep));

steps = (ag2-ag1)/(numags);
ags = ag1+(0:numags)'*steps;

end
end