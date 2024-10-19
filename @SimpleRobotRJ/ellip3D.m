function [xx,yy,zz] = ellip3D(r,n,rot,trans)

if length(n) ==1
    n = [1,1]*n;
end
phi = linspace(-pi/2, pi/2, n(1));
theta = linspace(0, 2*pi, n(2));

[Phi, Theta] = meshgrid(phi, theta);

%% abs(cos(pi/2)).^0.05 should be 0, but results 0.155 caused by float point
cP = cos(Phi);
sP = sin(Phi);
cT = cos(Theta);
sT = sin(Theta);

eps = 1e-10;
ind1 = any([abs(phi+pi/2)<=eps ; abs(phi-pi/2)<=eps]);
cP(:,ind1) = 0;
cP(:,phi==0) = 1;
sP(:,phi==0) = 0;
sP(:,ind1) = sign(sP(:,ind1));

ind1 = any([abs(theta-pi/2)<=eps ; abs(theta-3*pi/2)<=eps]);
ind2 = any([abs(theta)<=eps ; abs(theta-pi)<=eps ; abs(theta-2*pi)<=eps]);
cT(ind1,:) = 0;
cT(ind2,:) = sign(cT(ind2,:));
sT(ind2,:) = 0;
sT(ind1,:) = sign(sT(ind1,:));

%%
xx0 = r(1) * sign(cP) .* abs(cP) .* sign(cT) .* abs(cT);
yy0 = r(2) * sign(cP) .* abs(cP) .* sign(sT) .* abs(sT);
zz0 = r(3) * sign(sP) .* abs(sP);

xx = rot(1,1)*xx0 + rot(1,2)*yy0 + rot(1,3)*zz0;
yy = rot(2,1)*xx0 + rot(2,2)*yy0 + rot(2,3)*zz0;
zz = rot(3,1)*xx0 + rot(3,2)*yy0 + rot(3,3)*zz0;

xx = xx+trans(1);
yy = yy+trans(2);
zz = zz+trans(3);

end