function [robsc,jointc,jposit,toollinkc,tcpxyz] = linerobotc(robmodel,Tw,ftoolsize)
linknum = size(robmodel.links0,3);

linksc = HomTran(Tw(:,:,1:linknum),robmodel.links0);
robsc = [reshape(linksc(:,1:end-1,:),3,[]),linksc(:,end,end)];
jointc = HomTran(Tw(:,:,2:linknum),robmodel.joint0);
jposit = reshape(Tw(1:3,4,2:end-1),3,[]);
toollinkc = HomTran(Tw(:,:,linknum+1),robmodel.toollink0);
tcpxyz = HomTran(Tw(:,:,end),eye(3)*ftoolsize);
tcpxyz = [reshape(ones(3,1),1,1,[]).*Tw(1:3,4,end),reshape(tcpxyz,3,1,[])];

end
%%
function pts_new = HomTran(T,pts)
% Homogeneous Transformation for points, align size
% T: 4*4*p SE3 Matrix
% pts: 3*n*p Column Vectors
% if pts is 3*n and T is 4*4*p, each page, T(:,:,i)*pts

p = size(T,3);

if p==1
    pts_new = T(1:3,1:3)*pts + T(1:3,4);
else
    pts_new = pagemtimes(T(1:3,1:3,:),pts) + T(1:3,4,:);
end

end