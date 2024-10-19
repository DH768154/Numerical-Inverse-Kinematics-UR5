function RRMove(obj,T_goal,varargin)

dft_max_step = ones(1,obj.axisnum)*1/180*pi;
dft_max_step(4:end) = 2/180*pi;

p = inputParser;
addParameter(p,'max_step',dft_max_step)
addParameter(p,'maxloop',1000)
addParameter(p,'k',0.1)
addParameter(p,'errlim',[1/10/1000,0.1/180*pi])
addParameter(p,'transpose',false)
addParameter(p,'showframe',false)
addParameter(p,'warnmaxloop',true)
addParameter(p,'flexwrist',false)
parse(p,varargin{:});

max_step = p.Results.max_step;
maxloop = p.Results.maxloop;
Jtranspose = p.Results.transpose;
k = p.Results.k;
showframe = p.Results.showframe;
errlim = p.Results.errlim;
warnmaxloop = p.Results.warnmaxloop;
flexwrist = p.Results.flexwrist;

%%
if numel(T_goal)==3
    t = T_goal;
    T_goal = obj.cpose;
    T_goal(1:3,4) = t(:);
end
T_goal(1:3,1:3) = obj.ForceRotM(T_goal(1:3,1:3));

%% Show Start Stop Frame
for i = 1:3
    obj.simu.pltf0(i).XData = NaN;
    obj.simu.pltf0(i).YData = NaN;
    obj.simu.pltf0(i).ZData = NaN;
    obj.simu.pltf1(i).XData = NaN;
    obj.simu.pltf1(i).YData = NaN;
    obj.simu.pltf1(i).ZData = NaN;
end

if showframe
HomTran = @(T,pts) T(1:3,1:3)*pts + T(1:3,4);
f0 = HomTran(obj.cpose,eye(3)*obj.frametSize);
f0 = [reshape(ones(3,1),1,1,[]).*obj.cpose(1:3,4),reshape(f0,3,1,[])];
f1 = HomTran(T_goal,eye(3)*obj.frametSize);
f1 = [reshape(ones(3,1),1,1,[]).*T_goal(1:3,4),reshape(f1,3,1,[])];

for i = 1:3
    obj.simu.pltf0(i).XData = f0(1,:,i);
    obj.simu.pltf0(i).YData = f0(2,:,i);
    obj.simu.pltf0(i).ZData = f0(3,:,i);
    obj.simu.pltf1(i).XData = f1(1,:,i);
    obj.simu.pltf1(i).YData = f1(2,:,i);
    obj.simu.pltf1(i).ZData = f1(3,:,i);
end
end
%%
ag = obj.cjoint;
for i = 1:maxloop

    % matrix difference
    if flexwrist
        e = T_goal(1:3,4) - obj.cpose(1:3,4);
        c = norm(e)<=errlim(1);
    else
        [dp,dw,mag] = obj.diffSE3(T_goal,obj.cpose,'world');
        e = [dp;dw];
        c = mag(1)<=errlim(1) && mag(2)<=errlim(2);
    end


    % Stop Iteration
    
    if c
        break
    end

    J = obj.get_jacob0;

    if flexwrist
        J = J(1:3,:);
    end


    if Jtranspose
        dag = k * (J' * e)';
    else
        dag = k * (pinv(J) * e)';
    end
    dag = obj.bound2range(dag,max_step,true);
    ag = ag + dag;
    obj.MoveAbsJ(ag); 
end
if i==maxloop && warnmaxloop
    warning('Not Reach Target, Exceeded Max Loop')
end

for i = 1:3
    obj.simu.pltf0(i).XData = NaN;
    obj.simu.pltf0(i).YData = NaN;
    obj.simu.pltf0(i).ZData = NaN;
    obj.simu.pltf1(i).XData = NaN;
    obj.simu.pltf1(i).YData = NaN;
    obj.simu.pltf1(i).ZData = NaN;
end
end