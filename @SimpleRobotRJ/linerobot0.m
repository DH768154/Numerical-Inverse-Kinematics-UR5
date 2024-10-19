function robmodel = linerobot0(kinv,rotdirc,tcp,jsize,cirpts,offorder)

axisnum = length(rotdirc);
rotdirc = abs(rotdirc);
%% Links

links0 = NaN(3,4,axisnum+1);

for i = 1:axisnum+1
    offlink = diag(kinv(:,i));
    offlink = offlink(:,offorder(i,:));
    links0(:,:,i) = [zeros(3,1),cumsum(offlink,2)];
end
toollink0 = [zeros(3,1),tcp(1:3,4)];

%% Joit Circle

orit0 = eye(3);
joint0 = NaN(3,cirpts,axisnum);
for i = 1:axisnum
    joint0(:,:,i) = circle3([0;0;0],jsize(i),orit0(:,rotdirc(i)),cirpts);
end

%%

robmodel.links0 = links0;
robmodel.joint0 = joint0;
robmodel.toollink0 = toollink0;

end

%%
function circle_xyz = circle3(c,r,v,n)
% c: Center of the circle, 3*n
% r: Radius
% v: Normal vector of Circle, 3*n

rot_ag=linspace(0,2*pi,n); 

% Find 2 normal vector a and b, also normal to n
% [1,0,0] is just a ref, can use any vector
a=cross(v,[1 0 0]');
if ~any(a) 
    a=cross(v,[0 1 0]');
end
b=cross(v,a);

a=a/norm(a); 
b=b/norm(b);

circle_xyz = c+r*a.*cos(rot_ag)+r*b.*sin(rot_ag);
end