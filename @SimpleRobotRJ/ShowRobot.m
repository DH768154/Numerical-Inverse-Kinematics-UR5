function ShowRobot(obj,varargin)
% Show Robot

%% Check Angle
if isempty(varargin)
    ag = obj.cjoint;
else
    ag = varargin{1};
    obj.cjoint = ag;
end
inrange = obj.AgInRange(ag,obj.jrange);
if ~inrange
    disp('Check jrange property to get Joint Range')
    error('Angle is Not in Joint Range')
end

%% Floor size
[robsc,jointc,jposit,toollinkc,tcpxyz,~,obj.cpose] = obj.PrepareMotion(ag);
maxRg = sqrt(sum(obj.robmodel.links0(:,4,:).^2,1));
maxRg = sum(maxRg(2:end));


%% Plot
obj.simu.f_simu = figure('Name',obj.RobName,'NumberTitle','off');
%obj.simu.f_simu = figure;
% start/ stop frame single path
cl = [0.8500 0.3250 0.0980;0.4660 0.6740 0.1880;0 0.4470 0.7410];
obj.simu.pltf0 = gobjects(3,1);
obj.simu.pltf1 = gobjects(3,1);
for i = 1:3
    obj.simu.pltf0(i) = plot3(NaN,NaN,NaN,...
        '-','LineWidth',2,'Color',cl(i,:));hold on
    %obj.simu.pltf1(i) = obj.simu.pltf0(i);
    obj.simu.pltf1(i) = plot3(NaN,NaN,NaN,...
        '-','LineWidth',2,'Color',cl(i,:));hold on
end


% Joint Center
obj.simu.plt_jcenter = plot3(jposit{1}(1,:),jposit{1}(2,:),jposit{1}(3,:),...
    'ok','LineWidth',0.2,'MarkerFaceColor','k');hold on

% Joint (Circle)
obj.simu.plt_joint = gobjects(obj.axisnum,1);
for i = 1:obj.axisnum
    obj.simu.plt_joint(i) = fill3(jointc{1}(1,:,i),jointc{1}(2,:,i),jointc{1}(3,:,i)...
        ,'k','FaceAlpha',0.7,'LineStyle','-','LineWidth',1.5);
end

% Links
obj.simu.plt_linkyzx = plot3(robsc{1}(1,:),robsc{1}(2,:),robsc{1}(3,:)...
    ,'-','LineWidth',5,'Color',[0.2,0.2,0.2]);hold on
obj.simu.plt_linkyzx.Color(4) = 0.7;


% Tool
obj.simu.plt_tool = plot3(toollinkc{1}(1,:),toollinkc{1}(2,:),toollinkc{1}(3,:),...
    '-','LineWidth',2,'Color',[0 0.4470 0.7410]);hold on

%% TCP Frame
cl = [1,0,0;0.1961,0.8039,0.1961;0.1,0.1,1];
obj.simu.plt_tcpf = gobjects(3,1);
for i = 1:3
    obj.simu.plt_tcpf(i) = plot3(tcpxyz{1}(1,:,i),tcpxyz{1}(2,:,i),tcpxyz{1}(3,:,i),...
        '-','LineWidth',1.5,...
        'Color',cl(i,:));hold on
end

%% Path
obj.simu.plt_path = plot3(NaN,NaN,NaN,'-r','LineWidth',2);hold on
obj.simu.plt_path.Color(4) = 0.5;

%% Floor
[xx,yy] = meshgrid(linspace(-maxRg,maxRg,5),...
    linspace(-maxRg,maxRg,5));
zz = zeros(size(xx)) + obj.floorH;
obj.simu.pf = surf(xx,yy,zz,...
    'EdgeColor',[1,1,1]*0.6,'FaceColor',[1,1,1]*0.8,'FaceAlpha',0.3); hold on

%% Base Frame
if ~isnan(obj.frame0Size)
    obj.PlotFrame(obj.base,'scale',obj.frame0Size,'linewidth',2,'style','-',...
        'arrowsize',1/3,'colorind',1,'text','off');hold on
end
grid on; axis equal

%% Title: Joint Angle
obj.simu.tit = title(sprintf(['%7.2f',...
    repmat(' , %7.2f',1,obj.axisnum-1)],ag*180/pi),'fontname','Monospaced');


%% Elips
nn = NaN(2,2);
obj.simu.plt_elip = surf(nn,nn,nn,'EdgeColor',[0.9290 0.6940 0.1250],...
    'FaceColor','y','FaceAlpha',0.25,'AlphaData',0.5); hold on
ref_size = norm(obj.cpose(1:3,4))/5;
J0 = obj.get_jacob0(obj.home);
if any(strcmp(obj.elipType,{'pos','force'}))
    J0 = J0(1:3,:);
else
    J0 = J0(4:6,:);
end

s = abs(svd(J0));
if any(strcmp(obj.elipType,{'force','trq'}))
    s = 1./s;
end
s = max(s);

obj.elipSize = ref_size/s;
if obj.showellip
    [xx,yy,zz] = obj.get_ellipse('scale',obj.elipSize,'n',obj.elipN,'type',obj.elipType);
    obj.simu.plt_elip.XData = xx;
    obj.simu.plt_elip.YData = yy;
    obj.simu.plt_elip.ZData = zz;
end

%%
xlabel('X','FontWeight','bold');
ylabel('y','FontWeight','bold');
zlabel('z','FontWeight','bold')

k = [0.4,0.8;0.6,0.6;0,0.9];
limxyz = obj.est_range(k);
xlim(limxyz(1,:))
ylim(limxyz(2,:))
zlim(limxyz(3,:))

view(60,10)
set(obj.simu.f_simu,'Units','normalized','Position',[0.15,0.15,0.7,0.7])
end