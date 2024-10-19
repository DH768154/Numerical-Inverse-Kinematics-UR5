function Offs(obj,xyzval,varargin)

opt_ref = {'world','tool'};
validfun = @(x) any(validatestring(x,opt_ref));

dft_max_step = ones(1,obj.axisnum)*1/180*pi;
dft_max_step(4:end) = 2/180*pi;

p = inputParser;
addOptional(p,'ref','world',validfun)
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
max_step = [-1;1] * abs(max_step);
maxloop = p.Results.maxloop;
Jtranspose = p.Results.transpose;
k = p.Results.k;
errlim = p.Results.errlim;
showframe = p.Results.showframe;
ref = p.Results.ref;
warnmaxloop = p.Results.warnmaxloop;
flexwrist = p.Results.flexwrist;

%%
Tc = obj.cpose;
T = Tc;
if strcmp(ref,'world')
    
    T(1:3,4) = Tc(1:3,4)+xyzval(:);
else
    T(1:3,4) = Tc(1:3,1:3)*xyzval(:)+Tc(1:3,4);
end

obj.RRMove(T,'max_step',max_step,'maxloop',maxloop,...
    'transpose',Jtranspose,'k',k,'errlim',errlim,...
    'showframe',showframe,'warnmaxloop',warnmaxloop,'flexwrist',flexwrist)

end