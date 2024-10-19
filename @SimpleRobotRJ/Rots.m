function Rots(obj,theta,varargin)

opt_ref = {'world','tool'};
validfun1 = @(x) any(validatestring(x,opt_ref));
opt_dir = {'x','y','z'};
validfun2 = @(x) any(validatestring(x,opt_dir));

dft_max_step = ones(1,obj.axisnum)*1/180*pi;
dft_max_step(4:end) = 2/180*pi;

p = inputParser;
addRequired(p,'dir',validfun2)
addOptional(p,'ref','tool',validfun1)
addParameter(p,'max_step',dft_max_step)
addParameter(p,'maxloop',1000)
addParameter(p,'k',0.1)
addParameter(p,'errlim',[1/10/1000,0.1/180*pi])
addParameter(p,'transpose',false)
addParameter(p,'showframe',false)
addParameter(p,'warnmaxloop',true)
parse(p,varargin{:});

max_step = p.Results.max_step;
max_step = [-1;1] * abs(max_step);
maxloop = p.Results.maxloop;
Jtranspose = p.Results.transpose;
k = p.Results.k;
errlim = p.Results.errlim;
showframe = p.Results.showframe;
ref = p.Results.ref;
dir = p.Results.dir;
warnmaxloop = p.Results.warnmaxloop;

%%
R = rotxyz(theta,dir);

Tc = obj.cpose;
T = Tc;
if strcmp(ref,'world')
    T(1:3,1:3) = R*Tc(1:3,1:3);
else
    T(1:3,1:3) = Tc(1:3,1:3)*R;
end

obj.RRMove(T,'max_step',max_step,'maxloop',maxloop,...
    'transpose',Jtranspose,'k',k,'errlim',errlim,...
    'showframe',showframe,'warnmaxloop',warnmaxloop)

end


function R = rotxyz(theta,dir)
c=cos(theta); s=sin(theta);
if strcmp(dir,'x')
    R=[1,0,0;0,c,-s;0,s,c];
elseif strcmp(dir,'y')
    R=[c,0,s;0,1,0;-s,0,c];
elseif strcmp(dir,'z')
    R=[c,-s,0;s,c,0;0,0,1];
end
end