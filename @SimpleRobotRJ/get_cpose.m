function T_tcp = get_cpose(obj,varargin)
% if want to get current tcp pose, use obj.cpose is the same
if isempty(varargin)
    frame = 'tcp';
else
    frame = validatestring(varargin{1},{'tcp','tool0'});
end

if strcmp(frame,'tcp')
    T_tcp = obj.cpose;
else
    T_tcp = obj.cpose * obj.invSE3(obj.tcp);
end
end