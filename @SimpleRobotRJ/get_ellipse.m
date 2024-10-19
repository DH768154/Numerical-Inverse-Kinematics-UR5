function [xx,yy,zz] = get_ellipse(obj,varargin)
types = {'pos','orit','force','trq'};
validfun = @(x) any(validatestring(x,types));

p = inputParser;
addParameter(p,'n',20);
addParameter(p,'scale',0.1);
addParameter(p,'type','pos',validfun)
parse(p,varargin{:});
n = p.Results.n;
scale = p.Results.scale;
type = p.Results.type;
%%
J0 = obj.get_jacob0;
if any(strcmp(type,{'pos','force'}))
    J0 = J0(1:3,:);
else
    J0 = J0(4:6,:);
end

[u,s,~] = svd(J0);

if any(strcmp(type,{'force','trq'}))
    s = 1./diag(s);
else
    s = diag(s); 
end

%s = s/abs(max(s));
[xx,yy,zz] = obj.ellip3D(s*scale,n,u,obj.cpose(1:3,4));

end