function [robsc,jointc,jposit,toollinkc,tcpxyz,path,T_tcp] = PrepareMotion(obj,ags)

n = size(ags,1);
[robsc,jointc,jposit,toollinkc,tcpxyz] = deal(cell(n,1));
T_tcp = NaN(4,4,n);

for i = 1:n
    T = obj.FwdKine_RJ(ags(i,:),obj.rotdirc,obj.kinv,obj.tcp,obj.base,obj.flange);
    Tw = obj.cummult(T);
    T_tcp(:,:,i) = Tw(:,:,end);
    [robsc{i},jointc{i},jposit{i},toollinkc{i},tcpxyz{i}]...
        = obj.linerobotc(obj.robmodel,Tw,obj.frametSize);
end
path = reshape(T_tcp(1:3,4,:),3,[]);
end