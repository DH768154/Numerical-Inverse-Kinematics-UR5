function J0 = get_jacob0(obj,varargin)

if isempty(varargin)
    ag = obj.cjoint;
else
    ag = varargin{1};
end

axisnum = length(ag);
rotdirc = abs(obj.rotdirc);
dirs = sign(obj.rotdirc);
Tw = obj.fkine(ag);
Tw = Tw(:,:,[2:axisnum+1,axisnum+3]);

J0 = NaN(6,axisnum);
P = Tw(1:3,4,end);

for i=1:axisnum
    J0(4:6,i) = Tw(1:3,rotdirc(i),i) * dirs(i);
    J0(1:3,i) = cross(J0(4:6,i),P-Tw(1:3,4,i));
end

end