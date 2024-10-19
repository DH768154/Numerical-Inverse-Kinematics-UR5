function lwl()
Path = fileparts(mfilename('fullpath'));
data = imread(fullfile(Path,'w.x'));

s1 = reshape(data(1:2177304),1542,1412);
s2 = reshape(data(2177305:end),1468,2432);

imsize = round([1468/2432,1]*9000);

im = zeros([imsize,3]);

loc1 = [790,2445];
locend1 = size(s1)+loc1;
im(loc1(1):locend1(1)-1,loc1(2):locend1(2)-1,1)=s1;

loc2 = [3120,3125];
locend2 = size(s2)+loc2;
im(loc2(1):locend2(1)-1,loc2(2):locend2(2)-1,1)=s2;

f = figure('NumberTitle','off','Name',char([19968,20010,20581,24247,...
    30340,31038,20250,19981,24212,35813,21482,26377,19968,31181,22768,...
    38899]),'ToolBar','none','MenuBar','none','Visible','off');
imshow(im)
text(0.135,0.71,char([20320,32,32,32,20570,21040,21527,65311]),...
    'Units','normalized','FontUnits','normalized','FontSize',0.2,...
    'Color','w','FontName','SimSun')
text(0.08,0.285,char([20320,21548,32,32,32,32,32,20102,21527,65311]),...
    'Units','normalized','FontUnits','normalized','FontSize',0.2,...
    'Color','w','FontName','SimSun')
set(f,'Units','normalized','Position',[0.1500 0.1740 0.7000 0.6760],'Visible','on')

end