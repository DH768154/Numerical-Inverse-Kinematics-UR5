function Jog(obj)
axisnum = obj.axisnum;
f = figure('Name','Jog','ToolBar','none','MenuBar','none'...
    ,'Units','normalized','Position',[0,0.3,1/3,1/3],'NumberTitle','off');
tabgp = uitabgroup(f,'Units','normalized','Position',[0 0 1 1]);
tab1 = uitab(tabgp,'Title','    Joint    ',...
    'BackgroundColor','w');
tab2 = uitab(tabgp,'Title','    Cartesian    ',...
    'BackgroundColor','w');
%% Joint Space
pn = uipanel(tab1,'Units','normalized','Position',[0.05,0.16,0.9,0.8]);
g = 0.05;
h = (1-g*(axisnum+1))/axisnum;

bx = gobjects(axisnum, 1);
sld = gobjects(axisnum, 1);
for i = 1:axisnum
    uicontrol(pn,'Style','text','Units','normalized',...
        'String',['A' num2str(axisnum+1-i)],'Position',...
        [0,g*i+(i-1)*h-0.012-0.002,0.12,h],...
        'FontUnits','normalized','FontSize',0.6,'FontWeight','bold')
    bx(axisnum+1-i) = uicontrol(pn,'Style','edit','Units','normalized',...
        'String',num2str(obj.cjoint(axisnum+1-i)*180/pi,'%.2f'),...
        'Position',[0.12,g*i+(i-1)*h-0.002,0.15,h],...
        'FontUnits','normalized','FontSize',0.55,'FontWeight','bold');
    sld(axisnum+1-i) = uicontrol(pn,'Style','slider','Units','normalized',...
        'Min',obj.jrange(1,axisnum+1-i)*180/pi,...
        'Max',obj.jrange(2,axisnum+1-i)*180/pi,...
        'Position',[0.29,g*i+(i-1)*h-0.002,0.68,h],...
        'Value',obj.cjoint(axisnum+1-i)*180/pi,...
        'SliderStep',[0.5/diff(obj.jrange(:,axisnum+1-i))/180*pi,0.1]...
        ,'Callback',@(sld,event)obj.sliderMoving(sld,axisnum+1-i,bx(axisnum+1-i)));
    addlistener(sld(axisnum+1-i),'ContinuousValueChange',...
        @(sld,event)obj.sliderMoving(sld,axisnum+1-i,bx(axisnum+1-i)));
end

gp = [0.05,0.01,0.01,0.05];
w = (1-sum(gp))/3;
btn1 = uicontrol(tab1,'Style','pushbutton','Units','normalized',...
    'String','Move Robot','Position',[gp(1),0.03,w,0.1],...
    'FontUnits','normalized','FontSize',0.55,'FontWeight','bold');

btn2 = copyobj(btn1,tab1);
btn2.Position = [gp(1)+w+gp(2),0.03,w,0.1];
btn2.String = 'Go Home';

btn3 = copyobj(btn1,tab1);
btn3.Position = [sum(gp(1:3))+2*w,0.03,w,0.1];
btn3.String = 'Go Zero';

set(btn1,'callback',@(src, event) obj.jmovecallback());
set(btn2,'callback',@(src, event) obj.GoHome());
set(btn3,'callback',@(src, event) obj.GoZero());

%% WorkSpace

posstr = sprintf('[ %.2f , %.2f , %.2f ]',obj.cpose(1:3,4)*1000);

pn2 = uipanel(tab2,'Units','normalized','Position',[0.03,0.73,0.94,0.23]);
pn3 = uipanel(tab2,'Units','normalized','Position',[0.03,0.3973,0.94,0.3266]);
pn4 = uipanel(tab2,'Units','normalized','Position',[0.03,0.03,0.94,0.3572]);

txt1 = uicontrol(pn2,'Style','text','Units','normalized',...
    'String','POS [mm]:','Position',...
    [0.017,0.253,0.185,0.417],'HorizontalAlignment','left',...
    'FontUnits','normalized','FontSize',0.6,'FontWeight','bold');
posBox = uicontrol(pn2,'Style','edit','Units','normalized',...
    'String',posstr,'Position',...
    [0.207,0.3,0.6,0.4],...
    'FontUnits','normalized','FontSize',0.6);
btn4 = uicontrol(pn2,'Style','pushbutton','Units','normalized',...
    'String','Move','Position',[0.82,0.27,0.16,0.465],...
    'FontUnits','normalized','FontSize',0.55,'FontWeight','bold');

% accuracy 
txt2 = copyobj(txt1,pn4);
txt2.String = 'Max Err [mm,deg]: ';
txt2.Position = [0.049,0.5515,0.29,0.224];
accuracyBox = copyobj(posBox,pn4);
accuracyBox.String = '[ 0.1 , 0.1 ]';
accuracyBox.Position = [0.3544,0.5878,0.2275,0.23];

% step ag
txt3 = copyobj(txt2,pn4);
txt3.String = 'Max Angle: ';
txt3.Position = [0.049,0.17,0.162,0.224];
maxagBox = copyobj(accuracyBox,pn4);
fmt = ['[',repmat(' %.0f , ',1,axisnum-1),'%.0f ]'];
mxag = ones(1,axisnum);
mxag(4:end) = 2;
maxagBox.String = sprintf(fmt,mxag);
maxagBox.Position = [0.234241178612649,0.192973020725072,0.3486,0.23];

% num step
txt4 = copyobj(txt2,pn4);
txt4.String = '# Steps: ';
txt4.Position([1,3]) = [0.6685,0.12];
numstepBox = copyobj(accuracyBox,pn4);

numstepBox.String = '10';
numstepBox.Position([1,3]) = [0.82,0.1];

% ref
refBtn = uicontrol(pn4,'Style','checkbox','Units','normalized',...
    'String','Ref Tool Frame','Position',...
    [0.66,0.2,0.276,0.225],'HorizontalAlignment','left',...
    'FontUnits','normalized','FontSize',0.6,'FontWeight','bold','Value',0);

%%
strp = {'+ Tx','+ Ty','+ Tz','+ Rx','+ Ry','+ Rz'};
strn = {'- Tx','- Ty','- Tz','- Rx','- Ry','- Rz'};
gp = 0.02;
n = 6;
w = (1-gp*(n+1))/n;
h = (1-0.03*3)/2;
jogTBtn = gobjects(12,1);
for i = 1:6
    jogTBtn(i) = copyobj(btn1,pn3);
    jogTBtn(i).Position = [i*gp+(i-1)*w,2*0.03+h,w,h];
    jogTBtn(i).String = strp{i};
    jogTBtn(i).FontSize = 0.4;
    jogTBtn(i).FontName = 'Monospaced';
    jogTBtn(6+i) = copyobj(btn1,pn3);
    jogTBtn(6+i).Position = [i*gp+(i-1)*w,0.03,w,h];
    jogTBtn(6+i).String = strn{i}; 
    jogTBtn(6+i).FontSize = 0.4;
    jogTBtn(6+i).FontName = 'Monospaced';
end


%%
set(btn4,'callback',@(src, event) obj.gotoposcallback());

for i = 1:12
    set(jogTBtn(i),'callback',@(src, event) obj.lmovecallback(i));
end

%%
obj.guidata.f = f;
obj.guidata.bx = bx;
obj.guidata.posBox = posBox;

obj.guidata.accuracyBox = accuracyBox;
obj.guidata.maxagBox = maxagBox;
obj.guidata.numstepBox = numstepBox;
obj.guidata.refBtn = refBtn;

obj.guidata.sld = sld;

end

%%


