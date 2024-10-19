function T = FwdKine_RJ(ag,rotdirc,kinv,tcp,base,flange)

rotdirc = reshape(rotdirc,size(ag));
ag = ag.*sign(rotdirc);
rotdirc = abs(rotdirc);

axisnum = length(ag);
T = NaN(4,4,axisnum+3);

% Frame 1~6
for i = 1:axisnum
    T(:,:,i+1) = RnP(ag(i),rotdirc(i),kinv(:,i));
end

% Frame flange
T(:,:,i+2) = [flange,kinv(:,end);0,0,0,1];

% Frame tcp
T(:,:,end) = tcp;

% Frame base
T(:,:,1) = base;

end



%%
function T = RnP(theta,dir,P)

c=cos(theta); s=sin(theta);

if dir==1
    T=[1,0,0;0,c,-s;0,s,c];
elseif dir==2
    T=[c,0,s;0,1,0;-s,0,c];
elseif dir==3
    T=[c,-s,0;s,c,0;0,0,1];
end

T = [T,P;0,0,0,1];

end