function [dp,dw,mag] = diffSE3(F_ref,F_move,ref)
% Move F_move to F_ref
% ref: 'world' or 'self', transformation ref frame
%
% [dp,dw,mag] = diffmx(F_ref,F_move,'self')
% Simple version for SimpleRobRJ Class


if strcmpi(ref,'world')
    dR = F_ref(1:3,1:3)*F_move(1:3,1:3)';
    dp = F_ref(1:3,4) - F_move(1:3,4);
elseif strcmpi(ref,'self')
    dR = F_move(1:3,1:3)'*F_ref(1:3,1:3);
    dp = F_move(1:3,1:3)'*(F_ref(1:3,4) - F_move(1:3,4));
end

[dw,mag_w] = logSO3(dR);
mag = [norm(dp);mag_w];

end

%%
function [w,mag_w] = logSO3(R)
Vee3 = @(R) [R(3,2);R(1,3);R(2,1)];

ct = min(max((trace(R)-1)/2,-1),1);
mag_w = acos(ct);
w = 1/2/sin(mag_w)*Vee3(R-transpose(R));

if abs(mag_w)<1e-6 || ct>1
    w = [0,0,0]';
    mag_w=0;
elseif abs(mag_w-pi)<1e-6
    [~,~,V] = svd(eye(3)-R);
    w = V(:,end)/norm(V(:,end));
end

w = w.*mag_w;
end


