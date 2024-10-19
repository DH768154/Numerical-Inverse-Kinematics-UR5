function lmovecallback(obj,i)

refind = obj.guidata.refBtn.Value;

if refind==1
    ref = 'tool';
else
    ref = 'world';
end
max_step = eval(obj.guidata.maxagBox.String);
max_step = max_step*pi/180;
max_loop = round(eval(obj.guidata.numstepBox.String));

ind = [1,2,3];
dir = {'x','y','z'};
switch i
    case {1,2,3}
        t = [0;0;0];
        t(ind(i)) = 0.5;
        obj.Offs(t,ref,'maxloop',max_loop,'max_step',max_step,'warnmaxloop',false);
    case {7,8,9}
        t = [0;0;0];
        t(ind(i-6)) = -0.5;
        obj.Offs(t,ref,'maxloop',max_loop,'max_step',max_step,'warnmaxloop',false);
    case {4,5,6}
        obj.Rots( pi/2,dir{i-3},ref,'maxloop',max_loop,'max_step',max_step,...
            'k',1,'warnmaxloop',false);
    case {10,11,12}
        obj.Rots(-pi/2,dir{i-9},ref,'maxloop',max_loop,'max_step',max_step,...
            'k',1,'warnmaxloop',false);
end

end