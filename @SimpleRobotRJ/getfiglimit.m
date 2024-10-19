function [imxy,limz] = getfiglimit(obj)
limx = NaN(5,2); limy = limx; limz = limx;
[limx(1,:),limy(1,:),limz(1,:)] = getdatalim(obj.simu.plt_jcenter);
[limx(2,:),limy(2,:),limz(2,:)] = getarraylim(obj.simu.plt_joint);
[limx(3,:),limy(3,:),limz(3,:)] = getdatalim(obj.simu.plt_linkyzx);
[limx(4,:),limy(4,:),limz(4,:)] = getdatalim(obj.simu.plt_tool);
[limx(5,:),limy(5,:),limz(5,:)] = getarraylim(obj.simu.plt_tcpf);
lim = NaN(3,2);
lim(1,:) = [min(limx(:,1)),max(limx(:,2))];
lim(2,:) = [min(limy(:,1)),max(limy(:,2))];
lim(3,:) = [min(limz(:,1)),max(limz(:,2))];
lim(3,1) = min([lim(3,1),obj.base(3,4)-0.02]);

d = lim(:,2)-lim(:,1);
m = mean(lim,2);
imxy = m(1:2)+[-1,1].*max(d)*1.05/2;
limz = [lim(3,1)-max(d)*0.025,lim(3,1)+max(d)*1.05];

end

function [limx,limy,limz] = getdatalim(data)
xdata = data.XData;
ydata = data.YData;
zdata = data.ZData;
limx = [min(xdata),max(xdata)];
limy = [min(ydata),max(ydata)];
limz = [min(zdata),max(zdata)];
end

function [limx,limy,limz] = getarraylim(data)
xdata = arrayfun(@(x) x.XData, data, 'UniformOutput', false);
ydata = arrayfun(@(x) x.YData, data, 'UniformOutput', false);
zdata = arrayfun(@(x) x.ZData, data, 'UniformOutput', false);
limx = celllimit(xdata);
limy = celllimit(ydata);
limz = celllimit(zdata);
end

function limdata = celllimit(data)
limdata = cell2mat(cellfun(@(x) [min(x),max(x)],data,'UniformOutput',false));
limdata = [min(limdata(:,1)),max(limdata(:,2))];
end