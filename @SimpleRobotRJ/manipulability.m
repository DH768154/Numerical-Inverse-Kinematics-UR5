function [mu,v] = manipulability(obj,varargin)
% mu = [min(sig),min(sig)/max(sig),abs(det(J))]
% v is used to check which degree of freedom is limited, [trans,rot]
if isempty(varargin)
    ag = obj.cjoint;
else
    ag = varargin{1};
end
J = obj.get_jacob0(ag);
[u,s,~] = svd(J);
sig = diag(s);
v = u(:,end);
v = [v(1:3)/norm(v(1:3)),v(4:6)/norm(v(4:6))];
mu = [min(sig),min(sig)/max(sig),prod(sig)];
end