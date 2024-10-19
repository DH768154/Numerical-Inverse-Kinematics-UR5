classdef SimpleRobotRJ < handle

    %% 
    properties (Access = private)
        jsize
        cirpts
        offorder
        simu
        guidata
        allpath
    end

    %% Can read/write Outside
    properties 
        showag = true;
        tcptrace = true;
        keeptrace = false;
        
        jrange
        home
        floorH = 0;
        frame0Size = NaN;
        frametSize = NaN;

        elipSize = NaN;
        elipN = 20;
        showellip = false;
        elipType = 'pos';

        RobName = char([83,104,97,110,103,104,97,105,52,48,52]);
    end

    %% Can read only Outside
    properties (SetAccess = private)
        cjoint
        cpose
        axisnum
        robmodel
        kinv
        rotdirc
        tcp
        base
        flange        
    end

    methods
        %% Constructor
        function obj = SimpleRobotRJ(KinePara,DispPara)
%             [filepath,~,~] = fileparts(mfilename("fullpath"));
%             addpath(fullfile(filepath,'myfunc'))
            obj.jsize = DispPara.jsize;
            obj.cirpts = DispPara.cirpts;
            obj.offorder = DispPara.offorder;
            obj.kinv = KinePara.kinv;
            obj.rotdirc = KinePara.rotdirc;
            obj.tcp = KinePara.tcp;
            obj.base = KinePara.base;
            obj.flange = KinePara.flange;

            obj.axisnum = length(obj.rotdirc);
            obj.jrange = [-pi;pi] * ones(1,obj.axisnum);
            obj.cjoint = zeros(size(obj.rotdirc));
            obj.home = zeros(size(obj.rotdirc));
            obj.robmodel = obj.linerobot0(obj.kinv,obj.rotdirc,obj.tcp,...
                obj.jsize,obj.cirpts,obj.offorder);
        end

        %% Show Robot
        ShowRobot(obj,varargin);

        %% MoveAbsJ
        MoveAbsJ(obj,ags,varargin);

        %% jog
        Jog(obj);

        %% Go Home / Zero
        function GoHome(obj)
            obj.MoveAbsJ(obj.home);
        end

        function GoZero(obj)
            obj.MoveAbsJ(zeros(1,obj.axisnum));
        end    

        %% Calculate Forward Kinematics
        function Tw = fkine(obj,varargin)
            if isempty(varargin)
                ag = obj.cjoint;
            else
                ag = varargin{1};
            end
            T = obj.FwdKine_RJ(ag,obj.rotdirc,obj.kinv,obj.tcp,obj.base,obj.flange);
            Tw = obj.cummult(T);
        end

        %% Calculate Jacobian
        J0 = get_jacob0(obj,varargin);
        Jb = get_jacobb(obj,varargin);

        %% get tcp pose
        T_tcp = get_cpose(obj,varargin);

        %% manipulability
        [mu,v] = manipulability(obj,varargin);

        %% manipulability ellipsoid
        [xx,yy,zz] = get_ellipse(obj,varargin);
        
        %% resolved rate control
        RRMove(obj,T_goal,varargin);
        Offs(obj,xyzval,varargin)
        Rots(obj,theta,varargin)
        
        %% Zoom in / out
        function zoomin(obj)
            [limxy,limz] = obj.getfiglimit;  
            obj.simu.f_simu.Children.XLim = limxy(1,:);
            obj.simu.f_simu.Children.YLim = limxy(2,:);
            obj.simu.f_simu.Children.ZLim = limz;
        end

        function zoomout(obj)

            k = [0.9,0.9;0.9,0.9;0.5,1];
            limxyz = obj.est_range(k);
            obj.simu.f_simu.Children.XLim = limxyz(1,:);
            obj.simu.f_simu.Children.YLim = limxyz(2,:);
            obj.simu.f_simu.Children.ZLim = limxyz(3,:);


        end
    end

    %%
    % _______________________________________________________________________
    % ***********************************************************************
    
    methods (Access = private)
        %% prepare motion
        [robsc,jointc,jposit,toollinkc,tcpxyz,path,T_tcp] = PrepareMotion(obj,ags);

        %% Jog GUI Slider Moving Listener
        function sliderMoving(obj,sld,axis,bx)
            agi = sld.Value;
            bx.String = num2str(agi,'%.2f');
            ag = obj.cjoint;
            ag(axis) = agi/180*pi;
            obj.MoveAbsJ(ag,'jump');
        end   

        %% Jog GUI Button Callback

        function jmovecallback(obj)
            n = obj.axisnum;
            ag = NaN(1,n);
            for i = 1:n
                agi = str2double(obj.guidata.bx(i).String)/180*pi;
                ag(i) = min([max([agi,obj.jrange(1,i)]),obj.jrange(2,i)]);
                obj.guidata.bx(i).String = num2str(ag(i)*180/pi,'%.2f');
            end
            obj.MoveAbsJ(ag);
        end  

        %% Go to Pose Button Callback
        function gotoposcallback(obj)
            pose = eval(obj.guidata.posBox.String);
            maxag = eval(obj.guidata.maxagBox.String);
            maxerr = eval(obj.guidata.accuracyBox.String);
            obj.RRMove(pose/1000,'showframe',true,...
                'errlim',maxerr.*[1/1000,pi/180],'max_step',maxag*pi/180);
        end

        %% Linear Move Jog Buttom
        lmovecallback(obj,i);

        %% estimate working range
        function limxyz = est_range(obj,k)
            L = abs(reshape(obj.robmodel.links0(:,4,:),3,[]));
            maxL = max(L);
            r = sum(maxL(2:end))+norm(obj.tcp(1:3,4));
            b = L(:,1);
            
            limxyz = k.*[-1,1]*r+b+obj.base(1:3,4);
            limxyz(:,1) = limxyz(:,1)-(b+[0;0;0.02]);
            limxyz(3,1) = min([obj.floorH-0.02,limxyz(3,1)]);
        end

        %% get current figure limit
        [limxy,limz] = getfiglimit(obj);  
    end

    %%
    % _______________________________________________________________________
    % ***********************************************************************

    methods (Static)
        lwl();
    end
    methods (Static,Access = private)
        %% Angle in Joint Range
        function inrange = AgInRange(ag,jrange)
            if size(ag,1) == numel(ag)
                ag = ag';
            end
            inrange = all([all(ag>=jrange(1,:),2),all(ag<=jrange(2,:),2)],2);
        end
        
        %% bound2range
        function state = bound2range(state,range,keeprange)
            if ~keeprange
                ind_lb = range(1,:)>state;
                ind_ub = range(2,:)<state;
                state(ind_lb) = range(1,ind_lb);
                state(ind_ub) = range(2,ind_ub);
            else
                state = state/max([1,max(state./range,[],'all')]);
            end
        end
        %% Initialize Robot Model
        robmodel = linerobot0(kinv,rotdirc,tcp,jsize,cirpts,offorder);

        %% Update Robot Model
        [robsc,jointc,jposit,toollinkc,tcpxyz] = linerobotc(robmodel,Tw,ftoolsize);

        %% Matrix Difference
        [dp,dw,mag] = diffSE3(F_ref,F_move,ref);
        
        %% Force Rotation Matrix
        function R = ForceRotM(R)
            [u,~,v] = svd(R);
            R = u*v';
        end
        %% inv SE3
        function iT = invSE3(T)
            R = T(1:3,1:3);
            P = T(1:3,4);
            iT = [R',-R'*P;0,0,0,1];
        end
        %% other
        T_cum = cummult(T);
        T = FwdKine_RJ(ag,rotdirc,kinv,tcp,base,flange);
        pts_new = HomTran(T,pts);
        plt = PlotFrame(T,varargin);
        [xx,yy,zz] = ellip3D(r,n,rot,trans);

    end

    
    
end