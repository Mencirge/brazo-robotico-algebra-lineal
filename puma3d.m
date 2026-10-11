% This is PUMA3d.M, a 3D Matlab Kinematic model of a Puma robot located
% in the robotics lab of Walla Walla University.
% The file uses CAD data converted to Matlab using cad2matdemo.m, which 
% is located on the Mathworks central file exchange.
%
% This file is still being developed, for the latest version check the
% Mathworks central file exchange.
%
% Todo list:
% 1) optimize pumaANI, lots of stuff in loop that needs help.
% 2) move x, y, and z position to end effecter, not link 6 origin.
% 3) Toggle kinematics buttons on/off with inverse kinematics button.
% 4) Make this work with real time inverse kinematics.
% 5) Make the track on and off option better.
% 6) add other things that makes this program fun.
% 7) check for noplots and nogos
% 8) add some better "demos" for the button
% 9) Fix problem of more than one robot window.
%
function puma3d
% GUI kinematic demo for the Puma Robot.
% Robot geometry uses the CAD2MATDEMO code in the Mathworks file exchange
%
%%
loaddata
InitHome
%
% Create the push buttons: pos is: [left bottom width height]
demo = uicontrol(fig_1,'String','Demo','callback',@demo_button_press,...
    'Position',[20 5 60 20]);

rnd_demo = uicontrol(fig_1,'String','Random Move','callback',@rnd_demo_button_press,...
    'Position',[100 5 80 20]);

clr_trail = uicontrol(fig_1,'String','Clr Trail','callback',@clr_trail_button_press,...
    'Position',[200 5 60 20]);
%
home = uicontrol(fig_1,'String','Home','callback',@home_button_press,...
    'Position',[280 5 70 20]);
%
% Nuevos botones para crear y manipular la bolita:
btn_rnd_ball = uicontrol(fig_1,'String','Random Bolita','callback',@rnd_ball_button_press,...
    'Position',[360 5 95 20],'BackgroundColor',[0.85 0.98 0.85]);

btn_grasp_ball = uicontrol(fig_1,'String','Agarrar bolita','callback',@grasp_ball_button_press,...
    'Position',[460 5 95 20],'BackgroundColor',[1.0 0.92 0.82],'FontWeight','bold');

btn_release_ball = uicontrol(fig_1,'String','Soltar bolita','callback',@release_ball_button_press,...
    'Position',[560 5 80 20],'BackgroundColor',[1.0 0.90 0.90]);

btn_plot_traj = uicontrol(fig_1,'String','Gráfica \theta(t)','callback',@plot_traj_button_press,...
    'Position',[645 5 85 20]);
%
% Panel de Coordenadas de la Bolita
Ball_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 250 265 95],...
    'Title','Coordenadas de la Bolita (mm)','FontSize',10);

lbl_bx = uicontrol(Ball_p,'style','text','String','X:','Position',[6 48 16 18],'FontWeight','bold');
bx_edit = uicontrol(Ball_p,'style','edit','String','600','Position',[24 48 48 20]);

lbl_by = uicontrol(Ball_p,'style','text','String','Y:','Position',[76 48 16 18],'FontWeight','bold');
by_edit = uicontrol(Ball_p,'style','edit','String','350','Position',[94 48 48 20]);

lbl_bz = uicontrol(Ball_p,'style','text','String','Z:','Position',[146 48 16 18],'FontWeight','bold');
bz_edit = uicontrol(Ball_p,'style','edit','String','150','Position',[164 48 48 20]);

btn_set_xyz = uicontrol(Ball_p,'String','Poner','callback',@place_xyz_button_press,...
    'Position',[216 47 42 22],'BackgroundColor',[0.88 0.94 1.0],'FontWeight','bold');

btn_rnd_pnl = uicontrol(Ball_p,'String','Random Bolita','callback',@rnd_ball_button_press,...
    'Position',[10 12 115 24],'BackgroundColor',[0.88 0.98 0.88]);

btn_grasp_pnl = uicontrol(Ball_p,'String','Agarrar Bolita','callback',@grasp_ball_button_press,...
    'Position',[135 12 120 24],'BackgroundColor',[1.0 0.92 0.80],'FontWeight','bold');
%
% Panel de Información de Agarre y Trayectoria
Info_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 350 265 140],...
    'Title','Agarre y Trayectoria','FontSize',10);

info_cfg_lbl = uicontrol(Info_p,'style','text',...
    'String','Configuración:',...
    'Position',[8 96 85 18],'HorizontalAlignment','left','FontWeight','bold');
info_cfg_val = uicontrol(Info_p,'style','text',...
    'String','Ninguna',...
    'Position',[95 90 160 30],'HorizontalAlignment','left','ForegroundColor',[0 0.35 0.8]);

info_time_lbl = uicontrol(Info_p,'style','text',...
    'String','Tiempo total:',...
    'Position',[8 68 85 18],'HorizontalAlignment','left','FontWeight','bold');
info_time_val = uicontrol(Info_p,'style','text',...
    'String','-- s',...
    'Position',[95 68 160 18],'HorizontalAlignment','left');

info_dist_lbl = uicontrol(Info_p,'style','text',...
    'String','Distancia efector:',...
    'Position',[8 44 95 18],'HorizontalAlignment','left','FontWeight','bold');
info_dist_val = uicontrol(Info_p,'style','text',...
    'String','-- mm',...
    'Position',[105 44 150 18],'HorizontalAlignment','left');

info_status_lbl = uicontrol(Info_p,'style','text',...
    'String','Estado:',...
    'Position',[8 18 50 18],'HorizontalAlignment','left','FontWeight','bold');
info_status_val = uicontrol(Info_p,'style','text',...
    'String','Sin bolita',...
    'Position',[60 18 195 18],'HorizontalAlignment','left','ForegroundColor',[0.6 0.2 0.2]);
%
% Kinematics Panel
%
K_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 45 265 200],...
    'Title','Kinematics','FontSize',11);
%
%     Angle    Range                Default Name
%     Theta 1: 320 (-160 to 160)    90       Waist Joint  
%     Theta 2: 220 (-110 to 110)   -90       Shoulder Joint
%     Theta 3: 270 (-135 to 135)   -90       Elbow Joint    
%     Theta 4: 532 (-266 to 266)     0       Wrist Roll
%     Theta 5: 200 (-100 to 100)     0       Wrist Bend  
%     Theta 6: 532 (-266 to 266)     0       Wrist Swivel

t1_home = 90; % offset to define the "home" position as UP.
t2_home = -90;
t3_home = -90;

LD = 105; % Left, used to set the GUI.
HT = 18;  % Height
BT = 156; % Bottom
%%  GUI buttons for Theta 1.  pos is: [left bottom width height]
t1_slider = uicontrol(K_p,'style','slider',...
    'Max',160,'Min',-160,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t1_slider_button_press,...
    'Position',[LD BT 120 HT]);
t1_min = uicontrol(K_p,'style','text',...
    'String','-160',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t1_max = uicontrol(K_p,'style','text',...
    'String','+160',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t1_text = uicontrol(K_p,'style','text',...
    'String','t1',...
    'Position',[LD-100 BT 20 HT]);
% t1_text = uicontrol(K_p,'style','text',... % when matlab fixes uicontrol
%     'String','t1',...                      % for TeX, then I can use this.  
%     'Position',[LD-100 BT 20 HT]); % L, B, W, H
t1_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t1_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 2.
BT = 126;   % Bottom
t2_slider = uicontrol(K_p,'style','slider',...
    'Max',115,'Min',-115,'Value',0,...        % Mech. stop limits !
    'SliderStep',[0.05 0.2],...
    'callback',@t2_slider_button_press,...
    'Position',[LD BT 120 HT]);
t2_min = uicontrol(K_p,'style','text',...
    'String','-110',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t2_max = uicontrol(K_p,'style','text',...
    'String','+110',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t2_text = uicontrol(K_p,'style','text',...
    'String','t2',...
    'Position',[LD-100 BT 20 HT]);
t2_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t2_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 3.
BT = 96;   % Bottom
t3_slider = uicontrol(K_p,'style','slider',...
    'Max',135,'Min',-135,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t3_slider_button_press,...
    'Position',[LD BT 120 HT]);
t3_min = uicontrol(K_p,'style','text',...
    'String','-135',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t3_max = uicontrol(K_p,'style','text',...
    'String','+135',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t3_text = uicontrol(K_p,'style','text',...
    'String','t3',...
    'Position',[LD-100 BT 20 HT]);
t3_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t3_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 4.
BT = 66;   % Bottom
t4_slider = uicontrol(K_p,'style','slider',...
    'Max',266,'Min',-266,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t4_slider_button_press,...
    'Position',[LD BT 120 HT]);
t4_min = uicontrol(K_p,'style','text',...
    'String','-266',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t4_max = uicontrol(K_p,'style','text',...
    'String','+266',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t4_text = uicontrol(K_p,'style','text',...
    'String','t4',...
    'Position',[LD-100 BT 20 HT]);
t4_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t4_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 5.
BT = 36;   % Bottom
t5_slider = uicontrol(K_p,'style','slider',...
    'Max',100,'Min',-100,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t5_slider_button_press,...
    'Position',[LD BT 120 HT]);
t5_min = uicontrol(K_p,'style','text',...
    'String','-100',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t5_max = uicontrol(K_p,'style','text',...
    'String','+100',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t5_text = uicontrol(K_p,'style','text',...
    'String','t5',...
    'Position',[LD-100 BT 20 HT]);
t5_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t5_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 6.
BT = 6;   % Bottom
t6_slider = uicontrol(K_p,'style','slider',...
    'Max',266,'Min',-266,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t6_slider_button_press,...
    'Position',[LD BT 120 HT]);
t6_min = uicontrol(K_p,'style','text',...
    'String','-266',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t6_max = uicontrol(K_p,'style','text',...
    'String','+266',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t6_text = uicontrol(K_p,'style','text',...
    'String','t6',...
    'Position',[LD-100 BT 20 HT]);
t6_edit = uicontrol(K_p,'style','edit',...
    'String','0',...
    'callback',@t6_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%% Slider for Theta 1 motion.
%
    function t1_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t1_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t2old = T_Old(2); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(slider_value+t1_home,t2old,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 2 motion.
%
    function t2_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t2_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,slider_value+t2_home,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 3 motion.
    function t3_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t3_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,t2old,slider_value+t3_home,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 4 motion.
    function t4_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t4_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,t2old,t3old,slider_value,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 5 motion.
    function t5_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t5_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t6old = T_Old(6);
        pumaANI(t1old,t2old,t3old,t4old,slider_value,t6old,10,'n')
    end
%
%% Slider for Theta 6 motion.
    function t6_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t6_edit,'string',num2str(slider_value));
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t5old = T_Old(5);
        pumaANI(t1old,t2old,t3old,t4old,t5old,slider_value,10,'n')
    end
%
%% Edit box for Theta 1 motion.
%
    function t1_edit_button_press(h,dummy)
        user_entry = check_edit(h,-160,160,0,t1_edit);
        set(t1_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t2old = T_Old(2); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(user_entry+t1_home,t2old,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Edit box for Theta 2 motion.
%
    function t2_edit_button_press(h,dummy)
        user_entry = check_edit(h,-110,110,0,t2_edit);
        set(t2_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,user_entry+t2_home,t3old,t4old,t5old,t6old,10,'n')
    end
%% Edit box for Theta 3 motion.
%
    function t3_edit_button_press(h,dummy)
        user_entry = check_edit(h,-135,135,0,t3_edit);
        set(t3_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,user_entry+t3_home,t4old,t5old,t6old,10,'n')
    end
%%
%% Edit box for Theta 4 motion.
%
    function t4_edit_button_press(h,dummy)
        user_entry = check_edit(h,-266,266,0,t4_edit);
        set(t4_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,t3old,user_entry,t5old,t6old,10,'n')
    end
%% Edit box for Theta 5 motion.
%
    function t5_edit_button_press(h,dummy)
        user_entry = check_edit(h,-100,100,0,t5_edit);
        set(t5_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,t3old,t4old,user_entry,t6old,10,'n')
    end
%%
%% Edit box for Theta 6 motion.
%
    function t6_edit_button_press(h,dummy)
        user_entry = check_edit(h,-266,266,0,t6_edit);
        set(t6_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t5old = T_Old(5);
        %
        pumaANI(t1old,t2old,t3old,t4old,t5old,user_entry,10,'n')
    end
%%
    function user_entry = check_edit(h,min_v,max_v,default,h_edit)
        % This function will check the value typed in the text input box
        % against min and max values, and correct errors.
        %
        % h: handle of gui
        % min_v min value to check
        % max_v max value to check
        % default is the default value if user enters non number
        % h_edit is the edit value to update.
        %
        user_entry = str2double(get(h,'string'));
        if isnan(user_entry)
            errordlg(['You must enter a numeric value, defaulting to ',num2str(default),'.'],'Bad Input','modal')
            set(h_edit,'string',default);
            user_entry = default;
        end
        %
        if user_entry < min_v
            errordlg(['Minimum limit is ',num2str(min_v),' degrees, using ',num2str(min_v),'.'],'Bad Input','modal')
            user_entry = min_v;
            set(h_edit,'string',user_entry);
        end
        if user_entry > max_v
            errordlg(['Maximum limit is ',num2str(max_v),' degrees, using ',num2str(max_v),'.'],'Bad Input','modal')
            user_entry = max_v;
            set(h_edit,'string',user_entry);
        end
    end
%
%% Demo button's callback
    function demo_button_press(h,dummy)
        %
        % disp('pushed demo bottom');
        %         R = 500;
        %         x = 1000;
        n = 2;    % demo ani steps
        num = 30; % home to start, and end to home ani steps
        %         j = 1;
        %         M = 1000;
        for t = 0:.1:7*pi
            Px = 30*t*cos(t);
            Py = 1200-300*t*(t)/(50*pi);
            Pz = 30*t*sin(t);
            [theta1,theta2,theta3,theta4,theta5,theta6] = PumaIK(Px,Py,Pz);
            if t==0 %move to start of demo
                pumaANI(theta1,theta2,theta3-180,0,0,0,num,'n')
            end
            % Theta 4, 5 & 6 are zero due to plotting at wrist origen.
            pumaANI(theta1,theta2,theta3-180,0,0,0,n,'y')
            set(t1_edit,'string',num2str(round(theta1))); % Update slider and text.
            set(t1_slider,'Value',round(theta1));
            set(t2_edit,'string',num2str(round(theta2)));
            set(t2_slider,'Value',round(theta2));
            set(t3_edit,'string',num2str(round(theta3-180)));
            set(t3_slider,'Value',round(theta3-180));
        end
        gohome
%        pumaANI(90,-90,-90,0,0,0,num,'n')
    end
%
%
%%
    function home_button_press(h,dummy)
        %disp('pushed home bottom');
        gohome
    end
%
%%
    function clr_trail_button_press(h,dummy)
        %disp('pushed clear trail bottom');
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        if isappdata(0, 'traj_line_handle')
            tlh = getappdata(0, 'traj_line_handle');
            if ~isempty(tlh) && ishandle(tlh)
                set(tlh, 'XData', nan, 'YData', nan, 'ZData', nan);
            end
        end
    end
%
%
    function rnd_demo_button_press(h, dummy)
        %disp('pushed random demo bottom');
        % a = 10; b = 50; x = a + (b-a) * rand(5)
        %     Angle    Range                Default Name
        %     Theta 1: 320 (-160 to 160)    90       Waist Joint
        %     Theta 2: 220 (-110 to 110)    -90       Shoulder Joint
        %     Theta 3: 270 (-135 to 135)    -90       Elbow Joint
        %     Theta 4: 532 (-266 to 266)    0       Wrist Roll
        %     Theta 5: 200 (-100 to 100)    0       Wrist Bend
        %     Theta 6: 532 (-266 to 266)    0       Wrist Swival
        t1_home = 90; % offsets to define the "home" postition as UP.
        t2_home = -90;
        t3_home = -90;
        theta1 = -160 + 320*rand(1); % offset for home
        theta2 = -110 + 220*rand(1); % in the UP pos.
        theta3 = -135 + 270*rand(1);
        theta4 = -266 + 532*rand(1);
        theta5 = -100 + 200*rand(1);
        theta6 = -266 + 532*rand(1);
        n = 50;
        pumaANI(theta1+t1_home,theta2+t2_home,theta3+t3_home,theta4,theta5,theta6,n,'y')
        set(t1_edit,'string',num2str(round(theta1))); % Update slider and text.
        set(t1_slider,'Value',round(theta1));
        set(t2_edit,'string',num2str(round(theta2)));
        set(t2_slider,'Value',round(theta2));
        set(t3_edit,'string',num2str(round(theta3)));
        set(t3_slider,'Value',round(theta3));
        set(t4_edit,'string',num2str(round(theta4)));
        set(t4_slider,'Value',round(theta4));
        set(t5_edit,'string',num2str(round(theta5)));
        set(t5_slider,'Value',round(theta5));
        set(t6_edit,'string',num2str(round(theta6)));
        set(t6_slider,'Value',round(theta6));
    end
        
%% =========================================================================
%% NUEVAS FUNCIONES PARA CREAR, AGARRAR Y SOLTAR BOLITA
%% =========================================================================

    %% Función auxiliar para dibujar/actualizar la bolita en la escena 3D
    function spawn_ball_graphics(P_ball)
        handles = getappdata(0, 'patch_h');
        if ~isempty(handles) && ishandle(handles(1))
            ax_robot = ancestor(handles(1), 'axes');
            set(fig_1, 'CurrentAxes', ax_robot);
            hold(ax_robot, 'on');
        else
            ax_robot = gca;
        end
        
        % Limpiar bolita previa y rastro de trayectoria si existen
        old_ball = getappdata(0, 'ball_handle');
        if ~isempty(old_ball) && ishandle(old_ball)
            delete(old_ball);
        end
        old_traj = getappdata(0, 'traj_line_handle');
        if ~isempty(old_traj) && ishandle(old_traj)
            delete(old_traj);
        end
        setappdata(0, 'ball_grasped', 0);
        
        % Dibujar esfera 3D en los ejes del robot
        r_ball = 35;
        [sx, sy, sz] = sphere(20);
        ball_h = surf(ax_robot, r_ball*sx + P_ball(1), r_ball*sy + P_ball(2), r_ball*sz + P_ball(3), ...
            'FaceColor', [1.0, 0.35, 0.05], 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.8);
        
        setappdata(0, 'sphere_unit_x', r_ball * sx);
        setappdata(0, 'sphere_unit_y', r_ball * sy);
        setappdata(0, 'sphere_unit_z', r_ball * sz);
        setappdata(0, 'ball_handle', ball_h);
        setappdata(0, 'ball_pos', P_ball);
        
        % Actualizar cajas de texto de coordenadas
        set(bx_edit, 'string', num2str(round(P_ball(1))));
        set(by_edit, 'string', num2str(round(P_ball(2))));
        set(bz_edit, 'string', num2str(round(P_ball(3))));
    end

    %% Callback del botón "Random Bolita" (Paso 1: Generar aleatoria)
    function rnd_ball_button_press(h, dummy)
        set(info_status_val, 'String', 'Buscando posición alcanzable...', 'ForegroundColor', [0.8 0.4 0.0]);
        drawnow;
        
        [P_ball, ~] = generate_reachable_ball();
        spawn_ball_graphics(P_ball);
        
        set(info_status_val, 'String', sprintf('Bolita en [%.0f, %.0f, %.0f]. Presiona "Agarrar".', ...
            P_ball(1), P_ball(2), P_ball(3)), 'ForegroundColor', [0 0.5 0.2]);
    end

    %% Callback del botón "Poner" (Paso 1: Establecer coordenadas XYZ manuales)
    function place_xyz_button_press(h, dummy)
        x_val = str2double(get(bx_edit, 'string'));
        y_val = str2double(get(by_edit, 'string'));
        z_val = str2double(get(bz_edit, 'string'));
        
        if isnan(x_val) || isnan(y_val) || isnan(z_val)
            errordlg('Las coordenadas X, Y, Z deben ser valores numéricos.', 'Entrada Inválida');
            return;
        end
        
        P_cand = [x_val, y_val, z_val];
        sols = solve_puma_ik_all(P_cand);
        
        if isempty(sols)
            warndlg(sprintf('La posición [%.0f, %.0f, %.0f] no es alcanzable o colisiona con la base.', ...
                x_val, y_val, z_val), 'Punto Inalcanzable');
            set(info_status_val, 'String', 'Posición inalcanzable. Elige otra.', 'ForegroundColor', [0.8 0.2 0.2]);
            return;
        end
        
        spawn_ball_graphics(P_cand);
        set(info_status_val, 'String', sprintf('Bolita en [%.0f, %.0f, %.0f]. Presiona "Agarrar".', ...
            x_val, y_val, z_val), 'ForegroundColor', [0 0.5 0.2]);
    end

    %% Callback del botón "Agarrar bolita" (Paso 2: Planificar y Mover)
    function grasp_ball_button_press(h, dummy)
        try
            if ~isappdata(0, 'ball_pos') || isempty(getappdata(0, 'ball_pos'))
                warndlg('Primero debes crear o colocar una bolita (usa "Random Bolita" o "Poner").', 'Sin bolita');
                return;
            end
            
            if isappdata(0, 'ball_grasped') && getappdata(0, 'ball_grasped')
                msgbox('La bolita ya se encuentra agarrada en el efector final.', 'Aviso', 'help');
                return;
            end
            
            P_ball = getappdata(0, 'ball_pos');
            valid_solutions = solve_puma_ik_all(P_ball);
            
            if isempty(valid_solutions)
                errordlg('La bolita actual no tiene configuraciones válidas de agarre.', 'Error de cinemática');
                return;
            end
            
            set(info_status_val, 'String', 'Calculando agarre óptimo...', 'ForegroundColor', [0 0.4 0.8]);
            drawnow;
            
            % Obtener configuración articular actual (ángulos de usuario)
            T_Old = getappdata(0, 'ThetaOld');
            q_curr = [T_Old(1)-90, T_Old(2)+90, T_Old(3)+90, T_Old(4), T_Old(5), T_Old(6)];
            
            % Posición actual del efector final
            T_curr_fk = fk_puma(T_Old);
            P_ee_curr = T_curr_fk(1:3, 4) + T_curr_fk(1:3, 3) * 200;
            
            % Seleccionar solución que minimice sum(|Δθ|) con distancia como desempate
            best_idx = 1;
            min_joint_delta = Inf;
            min_ee_dist = Inf;
            
            for k = 1:length(valid_solutions)
                q_cand = valid_solutions(k).user_angles;
                joint_delta = sum(abs(q_cand - q_curr));
                ee_dist = norm(P_ee_curr - P_ball(:));
                
                if (joint_delta < min_joint_delta - 1e-4) || ...
                   (abs(joint_delta - min_joint_delta) <= 1e-4 && ee_dist < min_ee_dist)
                    min_joint_delta = joint_delta;
                    min_ee_dist = ee_dist;
                    best_idx = k;
                end
            end
            
            best_sol = valid_solutions(best_idx);
            set(info_status_val, 'String', 'Moviendo hacia la bolita...', 'ForegroundColor', [0 0.4 0.8]);
            drawnow;
            
            % Ejecutar trayectoria suave (polinomio quíntico) hacia la bolita
            animate_quintic_to_target(q_curr, best_sol.user_angles, P_ball, best_sol.desc);
        catch ME
            errordlg(ME.message, 'Error al agarrar');
            disp(ME.getReport());
        end
    end

    %% Callback del botón "Soltar bolita"
    function release_ball_button_press(h, dummy)
        if isappdata(0, 'ball_grasped') && getappdata(0, 'ball_grasped')
            setappdata(0, 'ball_grasped', 0);
            set(info_status_val, 'String', 'Bolita soltada. Lista para otra.', 'ForegroundColor', [0.1 0.6 0.2]);
            b_h = getappdata(0, 'ball_handle');
            if ~isempty(b_h) && ishandle(b_h)
                set(b_h, 'FaceColor', [0.2 0.85 0.45]); % verde suave para indicar liberada
            end
        else
            set(info_status_val, 'String', 'No hay bolita agarrada.', 'ForegroundColor', [0.6 0.2 0.2]);
        end
    end

    %% Callback para reabrir/visualizar la gráfica de ángulos vs tiempo
    function plot_traj_button_press(h, dummy)
        if isappdata(0, 'last_traj_data')
            traj_data = getappdata(0, 'last_traj_data');
            plot_joint_trajectory(traj_data.t, traj_data.q, traj_data.desc);
        else
            msgbox('Aún no se ha realizado ningún movimiento hacia una bolita.', 'Gráfica', 'help');
        end
    end

    %% Generador de bolita en espacio de trabajo alcanzable
    function [P_ball, valid_solutions] = generate_reachable_ball()
        % Genera repetidamente hasta encontrar una posición con soluciones válidas de IK
        max_attempts = 150;
        for att = 1:max_attempts
            r = 480 + (920 - 480) * rand(1);      % radio cilíndrico accesible (mm)
            phi = (-135 + 270 * rand(1)) * pi/180; % ángulo horizontal acimutal
            z = -120 + (500 - (-120)) * rand(1);   % altura z sobre la base (mm)
            
            cand_P = [r * cos(phi), r * sin(phi), z];
            
            sols = solve_puma_ik_all(cand_P);
            if ~isempty(sols)
                P_ball = cand_P;
                valid_solutions = sols;
                return;
            end
        end
        % Punto de respaldo garantizado si fallaran todos los intentos
        P_ball = [600, 350, 150];
        valid_solutions = solve_puma_ik_all(P_ball);
    end

    %% Solucionador completo de cinemática inversa con múltiples aproximaciones
    function valid_solutions = solve_puma_ik_all(P_ball)
        % Parámetros DH del Puma 762
        a2 = 650; a3 = 0; d3 = 190; d4 = 600; d_tool = 200;
        xb = P_ball(1); yb = P_ball(2); zb = P_ball(3);
        phi = atan2(yb, xb);
        
        % Definición de direcciones de aproximación del efector final (vector a = columna 3 de R)
        approaches = {};
        
        % 1. Superior (desde arriba apuntando hacia abajo): a = [0; 0; -1]
        a_sup = [0; 0; -1];
        o_sup = [-sin(phi); cos(phi); 0];
        n_sup = cross(o_sup, a_sup);
        approaches{end+1} = struct('name', 'aproximación superior', 'R', [n_sup, o_sup, a_sup]);
        
        % 2. Frontal (radial desde la base hacia la bolita): a = [cos(phi); sin(phi); 0]
        a_front = [cos(phi); sin(phi); 0];
        o_front = [0; 0; 1];
        n_front = cross(o_front, a_front);
        approaches{end+1} = struct('name', 'aproximación frontal', 'R', [n_front, o_front, a_front]);
        
        % 3. Lateral derecha (tangencial en plano horizontal): a = [-sin(phi); cos(phi); 0]
        a_lat1 = [-sin(phi); cos(phi); 0];
        o_lat1 = [0; 0; 1];
        n_lat1 = cross(o_lat1, a_lat1);
        approaches{end+1} = struct('name', 'aproximación lateral derecha', 'R', [n_lat1, o_lat1, a_lat1]);
        
        % 4. Lateral izquierda (tangencial opuesta): a = [sin(phi); -cos(phi); 0]
        a_lat2 = [sin(phi); -cos(phi); 0];
        o_lat2 = [0; 0; 1];
        n_lat2 = cross(o_lat2, a_lat2);
        approaches{end+1} = struct('name', 'aproximación lateral izquierda', 'R', [n_lat2, o_lat2, a_lat2]);
        
        valid_solutions = [];
        
        for ap = 1:length(approaches)
            app_name = approaches{ap}.name;
            R = approaches{ap}.R;
            
            % Centro de muñeca: Pw = P_ball - d_tool * a
            Pw = [xb; yb; zb] - d_tool * R(:, 3);
            xw = Pw(1); yw = Pw(2); zw = Pw(3);
            
            r2 = xw^2 + yw^2;
            if r2 < d3^2
                continue; % Inalcanzable por offset de hombro
            end
            
            K = (r2 + zw^2 - a2^2 - a3^2 - d3^2 - d4^2) / (2 * a2);
            if (a3^2 + d4^2) < K^2
                continue; % Fuera del alcance del brazo
            end
            
            % Probar configuraciones de hombro (sign1) y codo (sign3)
            for sign1 = [1, -1]
                for sign3 = [1, -1]
                    th1 = atan2(yw, xw) - atan2(d3, sign1 * sqrt(r2 - d3^2));
                    th3 = atan2(a3, d4) - atan2(K, sign3 * sqrt(a3^2 + d4^2 - K^2));
                    
                    c1 = cos(th1); s1 = sin(th1);
                    c3 = cos(th3); s3 = sin(th3);
                    
                    th23 = atan2((-a3 - a2*c3)*zw - (c1*xw + s1*yw)*(d4 - a2*s3), ...
                                 (a2*s3 - d4)*zw + (a3 + a2*c3)*(c1*xw + s1*yw));
                    th2 = th23 - th3;
                    
                    t1_deg = th1 * 180 / pi;
                    t2_deg = th2 * 180 / pi;
                    t3_deg = th3 * 180 / pi;
                    
                    % Matriz de orientación de los primeros 3 eslabones R_03
                    T01 = tmat(0, 0, 0, t1_deg);
                    T12 = tmat(-90, 0, 0, t2_deg);
                    T23 = tmat(0, a2, d3, t3_deg);
                    T03 = T01 * T12 * T23;
                    R03 = T03(1:3, 1:3);
                    
                    % Matriz de orientación de la muñeca: U = R_03' * R
                    U = R03' * R;
                    
                    s5_mag = sqrt(U(1,3)^2 + U(3,3)^2);
                    c5_val = U(2,3);
                    
                    % Configuraciones de muñeca: normal (s5 >= 0) e invertida (s5 <= 0)
                    wrist_configs = {
                        'muñeca normal', s5_mag, c5_val, atan2(U(3,3), -U(1,3)), atan2(-U(2,2), U(2,1));
                        'muñeca invertida', -s5_mag, c5_val, atan2(-U(3,3), U(1,3)), atan2(U(2,2), -U(2,1))
                    };
                    
                    for wc = 1:size(wrist_configs, 1)
                        w_name = wrist_configs{wc, 1};
                        s5_v = wrist_configs{wc, 2};
                        c5_v = wrist_configs{wc, 3};
                        th4_v = wrist_configs{wc, 4};
                        th6_v = wrist_configs{wc, 5};
                        
                        t5_deg = atan2(s5_v, c5_v) * 180 / pi;
                        t4_deg = th4_v * 180 / pi;
                        t6_deg = th6_v * 180 / pi;
                        
                        % Conversión a ángulos de usuario de la GUI
                        u1 = t1_deg - 90;
                        u2 = t2_deg + 90;
                        u3 = t3_deg + 90;
                        u4 = t4_deg;
                        u5 = t5_deg;
                        u6 = t6_deg;
                        
                        % Normalizar a rango [-180, 180]
                        u1 = mod(u1 + 180, 360) - 180;
                        u2 = mod(u2 + 180, 360) - 180;
                        u3 = mod(u3 + 180, 360) - 180;
                        u4 = mod(u4 + 180, 360) - 180;
                        u5 = mod(u5 + 180, 360) - 180;
                        u6 = mod(u6 + 180, 360) - 180;
                        
                        % Verificación estricta de límites mecánicos articulares
                        if (u1 < -160 || u1 > 160) || ...
                           (u2 < -110 || u2 > 110) || ...
                           (u3 < -135 || u3 > 135) || ...
                           (u4 < -266 || u4 > 266) || ...
                           (u5 < -100 || u5 > 100) || ...
                           (u6 < -266 || u6 > 266)
                            continue;
                        end
                        
                        % Verificación de colisión con la base y el suelo
                        % 1. El codo y la muñeca deben estar por encima del piso (-1000 mm)
                        P_elbow = T01 * T12 * [0; a2; 0; 1];
                        if P_elbow(3) < -380 || Pw(3) < -380
                            continue;
                        end
                        % 2. La muñeca no debe penetrar el cilindro de la base (r < 300 mm y z < 0 mm)
                        if sqrt(xw^2 + yw^2) < 310 && Pw(3) < 0
                            continue;
                        end
                        
                        elbow_str = 'codo arriba';
                        if sign3 == -1
                            elbow_str = 'codo abajo';
                        end
                        
                        sol_desc = sprintf('%s, %s, %s', elbow_str, w_name, app_name);
                        
                        sol_entry.desc = sol_desc;
                        sol_entry.user_angles = [u1, u2, u3, u4, u5, u6];
                        sol_entry.dh_angles = [u1+90, u2-90, u3-90, u4, u5, u6];
                        
                        if isempty(valid_solutions)
                            valid_solutions = sol_entry;
                        else
                            valid_solutions(end+1) = sol_entry;
                        end
                    end
                end
            end
        end
    end

    %% Función de animación suave (polinomio quíntico) hacia el agarre
    function animate_quintic_to_target(q_start, q_target, P_ball, config_desc)
        % Obtener ejes del robot de forma robusta y fijar como activos
        handles = getappdata(0,'patch_h');
        L1 = handles(1); L2 = handles(2); L3 = handles(3);
        L4 = handles(4); L5 = handles(5); L6 = handles(6);
        L7 = handles(7);
        ax_robot = ancestor(L1, 'axes');
        set(fig_1, 'CurrentAxes', ax_robot);
        hold(ax_robot, 'on');
        
        % Calcular duración total basada en el máximo desplazamiento articular
        max_dq = max(abs(q_target - q_start));
        T_total = max(2.5, max_dq / 35.0); % velocidad articular cómoda ~35 deg/s
        
        N = max(45, round(T_total * 25)); % pasos a ~25 fps
        t = linspace(0, T_total, N);
        tau = t / T_total;
        
        % Polinomio quíntico s(tau) con velocidad y aceleración inicial/final cero
        s = 10 * (tau.^3) - 15 * (tau.^4) + 6 * (tau.^5);
        
        q_traj = zeros(N, 6);
        for j = 1:6
            q_traj(:, j) = q_start(j) + s' * (q_target(j) - q_start(j));
        end
        
        % Guardar datos de trayectoria para la gráfica
        traj_data.t = t;
        traj_data.q = q_traj;
        traj_data.desc = config_desc;
        setappdata(0, 'last_traj_data', traj_data);
        
        % Preparar la línea 3D de trayectoria del efector final
        old_line = getappdata(0, 'traj_line_handle');
        if ~isempty(old_line) && ishandle(old_line)
            delete(old_line);
        end
        traj_line_h = plot3(ax_robot, nan, nan, nan, 'Color', [0.95 0.15 0.15], 'LineWidth', 2.5);
        setappdata(0, 'traj_line_handle', traj_line_h);
        
        % Parámetros geométricos para FK
        a2 = 650; a3 = 0; d3 = 190; d4 = 600; d_tool = 200;
        s1_cad = getappdata(0,'Link1_data');
        s2_cad = getappdata(0,'Link2_data');
        s3_cad = getappdata(0,'Link3_data');
        s4_cad = getappdata(0,'Link4_data');
        s5_cad = getappdata(0,'Link5_data');
        s6_cad = getappdata(0,'Link6_data');
        s7_cad = getappdata(0,'Link7_data');
        
        tip_pts = zeros(3, N);
        dt_frame = T_total / N;
        
        % Bucle de animación suave
        for k = 1:N
            q_k = q_traj(k, :);
            t1 = q_k(1) + 90;
            t2 = q_k(2) - 90;
            t3 = q_k(3) - 90;
            t4 = q_k(4);
            t5 = q_k(5);
            t6 = q_k(6);
            
            T_01 = tmat(0, 0, 0, t1);
            T_12 = tmat(-90, 0, 0, t2);
            T_23 = tmat(0, a2, d3, t3);
            T_34 = tmat(-90, a3, d4, t4);
            T_45 = tmat(90, 0, 0, t5);
            T_56 = tmat(-90, 0, 0, t6);
            
            T_02 = T_01 * T_12;
            T_03 = T_02 * T_23;
            T_04 = T_03 * T_34;
            T_05 = T_04 * T_45;
            T_06 = T_05 * T_56;
            
            Link1 = s1_cad.V1;
            Link2 = (T_01 * s2_cad.V2')';
            Link3 = (T_02 * s3_cad.V3')';
            Link4 = (T_03 * s4_cad.V4')';
            Link5 = (T_04 * s5_cad.V5')';
            Link6 = (T_05 * s6_cad.V6')';
            Link7 = (T_06 * s7_cad.V7')';
            
            set(L1,'vertices',Link1(:,1:3));
            set(L2,'vertices',Link2(:,1:3));
            set(L3,'vertices',Link3(:,1:3));
            set(L4,'vertices',Link4(:,1:3));
            set(L5,'vertices',Link5(:,1:3));
            set(L6,'vertices',Link6(:,1:3));
            set(L7,'vertices',Link7(:,1:3));
            
            % Posición del efector final (punta de la herramienta)
            P_tip = T_06 * [0; 0; d_tool; 1];
            tip_pts(:, k) = P_tip(1:3);
            
            % Actualizar línea 3D de la trayectoria
            set(traj_line_h, 'XData', tip_pts(1, 1:k), ...
                             'YData', tip_pts(2, 1:k), ...
                             'ZData', tip_pts(3, 1:k));
            
            % Actualizar sliders y cajas de texto de la GUI con límites seguros
            set(t1_slider, 'Value', max(-160, min(160, round(q_k(1))))); set(t1_edit, 'string', num2str(round(q_k(1))));
            set(t2_slider, 'Value', max(-110, min(110, round(q_k(2))))); set(t2_edit, 'string', num2str(round(q_k(2))));
            set(t3_slider, 'Value', max(-135, min(135, round(q_k(3))))); set(t3_edit, 'string', num2str(round(q_k(3))));
            set(t4_slider, 'Value', max(-266, min(266, round(q_k(4))))); set(t4_edit, 'string', num2str(round(q_k(4))));
            set(t5_slider, 'Value', max(-100, min(100, round(q_k(5))))); set(t5_edit, 'string', num2str(round(q_k(5))));
            set(t6_slider, 'Value', max(-266, min(266, round(q_k(6))))); set(t6_edit, 'string', num2str(round(q_k(6))));
            
            drawnow;
            pause(min(0.02, max(0.005, dt_frame * 0.4)));
        end
        
        % Calcular distancia total recorrida por el efector final
        diff_pts = diff(tip_pts, 1, 2);
        dist_traveled = sum(sqrt(sum(diff_pts.^2, 1)));
        
        % Actualizar interfaz con datos del movimiento
        set(info_cfg_val, 'String', config_desc);
        set(info_time_val, 'String', sprintf('%.2f s', T_total));
        set(info_dist_val, 'String', sprintf('%.1f mm', dist_traveled));
        set(info_status_val, 'String', 'Sujetando bolita (Agarrada)', 'ForegroundColor', [0.1 0.55 0.15]);
        
        % Simular agarre: vincular la bolita al marco del efector final T_06
        setappdata(0, 'ball_grasped', 1);
        b_loc = T_06 \ [P_ball(1); P_ball(2); P_ball(3); 1];
        setappdata(0, 'ball_local_pos', b_loc);
        
        % Actualizar pose guardada
        final_dh = [q_target(1)+90, q_target(2)-90, q_target(3)-90, q_target(4), q_target(5), q_target(6)];
        setappdata(0, 'ThetaOld', final_dh);
        
        % Si la ventana de gráfica ya estaba abierta por el usuario, actualizarla
        fig_traj = findobj('type', 'figure', 'name', 'Trayectoria Articular: Ángulos vs Tiempo');
        if ~isempty(fig_traj) && ishandle(fig_traj)
            plot_joint_trajectory(t, q_traj, config_desc);
        end
    end

    %% Gráfica de trayectoria articular vs tiempo
    function plot_joint_trajectory(t, q_traj, config_desc)
        fig_traj = findobj('type', 'figure', 'name', 'Trayectoria Articular: Ángulos vs Tiempo');
        if isempty(fig_traj)
            fig_traj = figure('Name', 'Trayectoria Articular: Ángulos vs Tiempo', ...
                'NumberTitle', 'off', 'Color', [0.96 0.96 0.96], 'Position', [100 120 620 400]);
        else
            figure(fig_traj);
            clf(fig_traj);
        end
        
        hold on;
        p1 = plot(t, q_traj(:, 1), 'r-', 'LineWidth', 2);
        p2 = plot(t, q_traj(:, 2), 'g-', 'LineWidth', 2);
        p3 = plot(t, q_traj(:, 3), 'b-', 'LineWidth', 2);
        p4 = plot(t, q_traj(:, 4), 'm--', 'LineWidth', 2);
        p5 = plot(t, q_traj(:, 5), 'c-', 'LineWidth', 2);
        p6 = plot(t, q_traj(:, 6), 'k--', 'LineWidth', 2);
        grid on;
        
        xlabel('Tiempo (s)', 'FontSize', 10, 'FontWeight', 'bold');
        ylabel('Ángulo articular (grados)', 'FontSize', 10, 'FontWeight', 'bold');
        title(['Trayectoria Articular (Quíntica) - ', config_desc], 'FontSize', 10, 'FontWeight', 'bold');
        legend([p1, p2, p3, p4, p5, p6], ...
            {'\theta_1 (Cintura)', '\theta_2 (Hombro)', '\theta_3 (Codo)', ...
             '\theta_4 (Muñeca Roll)', '\theta_5 (Muñeca Bend)', '\theta_6 (Muñeca Swivel)'}, ...
            'Location', 'best');
        hold off;
        
        % Regresar el foco a la ventana 3D del robot
        if ishandle(fig_1)
            figure(fig_1);
        end
    end

    %% Cinemática directa auxiliar
    function T = fk_puma(T_angles)
        a2 = 650; a3 = 0; d3 = 190; d4 = 600;
        T01 = tmat(0, 0, 0, T_angles(1));
        T12 = tmat(-90, 0, 0, T_angles(2));
        T23 = tmat(0, a2, d3, T_angles(3));
        T34 = tmat(-90, a3, d4, T_angles(4));
        T45 = tmat(90, 0, 0, T_angles(5));
        T56 = tmat(-90, 0, 0, T_angles(6));
        T = T01 * T12 * T23 * T34 * T45 * T56;
    end

%%
%Here are the functions used for this robot example:
%
%%
% When called this function will simply initialize a plot of the Puma 762
% robot by plotting it in it's home orientation and setting the current
% angles accordingly.
    function gohome()
        pumaANI(90,-90,-90,0,0,0,20,'n') % show it animate home
        %PumaPOS(90,-90,-90,0,0,0)  %drive it home, no animate.
        set(t1_edit,'string','0');
        set(t1_slider,'Value',0);  %At the home position, so all
        set(t2_edit,'string','0');   %sliders and input boxes = 0. 
        set(t2_slider,'Value',0);
        set(t3_edit,'string','0');
        set(t3_slider,'Value',0);
        set(t4_edit,'string','0');
        set(t4_slider,'Value',0);
        set(t5_edit,'string','0');
        set(t5_slider,'Value',0);
        set(t6_edit,'string','0');
        set(t6_slider,'Value',0);
        setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
    end
%%
% This function will load the 3D CAD data.
%
function loaddata
% Loads all the link data from file linksdata.mat.
% This data comes from a Pro/E 3D CAD model and was made with cad2matdemo.m
% from the file exchange.  All link data manually stored in linksdata.mat
[linkdata]=load('linksdata.mat','s1','s2', 's3','s4','s5','s6','s7','A1');

%Place the robot link 'data' in a storage area
setappdata(0,'Link1_data',linkdata.s1);
setappdata(0,'Link2_data',linkdata.s2);
setappdata(0,'Link3_data',linkdata.s3);
setappdata(0,'Link4_data',linkdata.s4);
setappdata(0,'Link5_data',linkdata.s5);
setappdata(0,'Link6_data',linkdata.s6);
setappdata(0,'Link7_data',linkdata.s7);
setappdata(0,'Area_data',linkdata.A1);
end
%
%%
% Use forward kinematics to place the robot in a specified configuration.
%
    function PumaPOS(theta1,theta2,theta3,theta4,theta5,theta6)

        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        s5 = getappdata(0,'Link5_data');
        s6 = getappdata(0,'Link6_data');
        s7 = getappdata(0,'Link7_data');
        A1 = getappdata(0,'Area_data');
        %
        a2 = 650;
        a3 = 0;
        d3 = 190;
        d4 = 600;
        Px = 5000;
        Py = 5000;
        Pz = 5000;

        t1 = theta1; 
        t2 = theta2; 
        t3 = theta3 %-180;  
        t4 = theta4; 
        t5 = theta5; 
        t6 = theta6; 
        %
        % Forward Kinematics
        T_01 = tmat(0, 0, 0, t1);
        T_12 = tmat(-90, 0, 0, t2);
        T_23 = tmat(0, a2, d3, t3);
        T_34 = tmat(-90, a3, d4, t4);
        T_45 = tmat(90, 0, 0, t5);
        T_56 = tmat(-90, 0, 0, t6);

        %T_01 = T_01;
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;
        %
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';

        handles = getappdata(0,'patch_h');           %
        L1 = handles(1);
        L2 = handles(2);
        L3 = handles(3);
        L4 = handles(4);
        L5 = handles(5);
        L6 = handles(6);
        L7 = handles(7);
        %
        set(L1,'vertices',Link1(:,1:3),'facec', [0.717,0.116,0.123]);
        set(L1, 'EdgeColor','none');
        set(L2,'vertices',Link2(:,1:3),'facec', [0.216,1,.583]);
        set(L2, 'EdgeColor','none');
        set(L3,'vertices',Link3(:,1:3),'facec', [0.306,0.733,1]);
        set(L3, 'EdgeColor','none');
        set(L4,'vertices',Link4(:,1:3),'facec', [1,0.542,0.493]);
        set(L4, 'EdgeColor','none');
        set(L5,'vertices',Link5(:,1:3),'facec', [0.216,1,.583]);
        set(L5, 'EdgeColor','none');
        set(L6,'vertices',Link6(:,1:3),'facec', [1,1,0.255]);
        set(L6, 'EdgeColor','none');
        set(L7,'vertices',Link7(:,1:3),'facec', [0.306,0.733,1]);
        set(L7, 'EdgeColor','none');
        % Si la bolita está agarrada, actualizar su posición en la escena 3D con T_06
        if isappdata(0, 'ball_grasped') && getappdata(0, 'ball_grasped')
            b_h = getappdata(0, 'ball_handle');
            if ~isempty(b_h) && ishandle(b_h)
                b_loc = getappdata(0, 'ball_local_pos');
                if isempty(b_loc)
                    b_loc = [0; 0; 200; 1];
                end
                b_w = T_06 * b_loc;
                ux = getappdata(0, 'sphere_unit_x');
                uy = getappdata(0, 'sphere_unit_y');
                uz = getappdata(0, 'sphere_unit_z');
                if ~isempty(ux)
                    set(b_h, 'XData', ux + b_w(1), 'YData', uy + b_w(2), 'ZData', uz + b_w(3));
                end
            end
        end
    end
%%
% This function computes the Inverse Kinematics for the Puma 762 robot
% given X,Y,Z coordinates for a point in the workspace. Note: The IK are
% computed for the origin of Coordinate systems 4,5 & 6.
    function [theta1,theta2,theta3,theta4,theta5,theta6] = PumaIK(Px,Py,Pz)
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
        sign1 = 1;
        sign3 = 1;
        nogo = 0;
        noplot = 0;
        % Because the sqrt term in theta1 & theta3 can be + or - we run through
        % all possible combinations (i = 4) and take the first combination that
        % satisfies the joint angle constraints.
        while nogo == 0;
            for i = 1:1:4
                if i == 1
                    sign1 = 1;
                    sign3 = 1;
                elseif i == 2
                    sign1 = 1;
                    sign3 = -1;
                elseif i == 3
                    sign1 = -1;
                    sign3 = 1;
                else
                    sign1 = -1;
                    sign3 = -1;
                end
                a2 = 650;
                a3 = 0;
                d3 = 190;
                d4 = 600;
                rho = sqrt(Px^2+Py^2);
                phi = atan2(Py,Px);
                K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
                c4 = cos(theta4);
                s4 = sin(theta4);
                c5 = cos(theta5);
                s5 = sin(theta5);
                c6 = cos(theta6);
                s6 = sin(theta6);
                theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));

                c1 = cos(theta1);
                s1 = sin(theta1);
                theta3 = (atan2(a3,d4)-atan2(K,sign3*sqrt(a3^2+d4^2-K^2)));

                c3 = cos(theta3);
                s3 = sin(theta3);
                t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
                theta2 = (t23 - theta3);

                c2 = cos(theta2);
                s2 = sin(theta2);
                s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
                c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);
                r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
                r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
                r33 = s23*c4*s5 - c23*c5;
                theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);

                r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
                r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
                r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
                s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
                c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
                theta5 = atan2(s5,c5);

                s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
                c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
                theta6 = atan2(s6,c6);

                theta1 = theta1*180/pi;
                theta2 = theta2*180/pi;
                theta3 = theta3*180/pi;
                theta4 = theta4*180/pi;
                theta5 = theta5*180/pi;
                theta6 = theta6*180/pi;
                if theta2>=160 && theta2<=180
                    theta2 = -theta2;
                end

                if theta1<=160 && theta1>=-160 && (theta2<=20 && theta2>=-200) && theta3<=45 && theta3>=-225 && theta4<=266 && theta4>=-266 && theta5<=100 && theta5>=-100 && theta6<=266 && theta6>=-266
                    nogo = 1;
                    theta3 = theta3+180;
                    break
                end
                if i == 4 && nogo == 0
                    h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                    waitfor(h);
                    nogo = 1;
                    noplot = 1;
                    break
                end
            end
        end
    end
%
%% 
    function pumaANI(theta1,theta2,theta3,theta4,theta5,theta6,n,trail)
        % This function will animate the Puma 762 robot given joint angles.
        % n is number of steps for the animation
        % trail is 'y' or 'n' (n = anything else) for leaving a trail.
        %
        %disp('in animate');
        a2 = 650; %D-H paramaters
        a3 = 0;
        d3 = 190;
        d4 = 600;
        % Err2 = 0;
        %
        ThetaOld = getappdata(0,'ThetaOld');
        %
        theta1old = ThetaOld(1);
        theta2old = ThetaOld(2);
        theta3old = ThetaOld(3);
        theta4old = ThetaOld(4);
        theta5old = ThetaOld(5);
        theta6old = ThetaOld(6);
        %
        t1 = linspace(theta1old,theta1,n); 
        t2 = linspace(theta2old,theta2,n); 
        t3 = linspace(theta3old,theta3,n);% -180;  
        t4 = linspace(theta4old,theta4,n); 
        t5 = linspace(theta5old,theta5,n); 
        t6 = linspace(theta6old,theta6,n); 

        n = length(t1);
        for i = 2:1:n
            % Forward Kinematics
            %
            T_01 = tmat(0, 0, 0, t1(i));
            T_12 = tmat(-90, 0, 0, t2(i));
            T_23 = tmat(0, a2, d3, t3(i));
            T_34 = tmat(-90, a3, d4, t4(i));
            T_45 = tmat(90, 0, 0, t5(i));
            T_56 = tmat(-90, 0, 0, t6(i));

% 
%             %     T_67 = [   1            0      0 0
%             %                0            1      0 0
%             %                0            0      1 188
%             %                0            0      0 1];

            %T_01 = T_01;  % it is, but don't need to say so.
            T_02 = T_01*T_12;
            T_03 = T_02*T_23;
            T_04 = T_03*T_34;
            T_05 = T_04*T_45;
            T_06 = T_05*T_56;
            %     T_07 = T_06*T_67;
            %
            s1 = getappdata(0,'Link1_data');
            s2 = getappdata(0,'Link2_data');
            s3 = getappdata(0,'Link3_data');
            s4 = getappdata(0,'Link4_data');
            s5 = getappdata(0,'Link5_data');
            s6 = getappdata(0,'Link6_data');
            s7 = getappdata(0,'Link7_data');
            %A1 = getappdata(0,'Area_data');

            Link1 = s1.V1;
            Link2 = (T_01*s2.V2')';
            Link3 = (T_02*s3.V3')';
            Link4 = (T_03*s4.V4')';
            Link5 = (T_04*s5.V5')';
            Link6 = (T_05*s6.V6')';
            Link7 = (T_06*s7.V7')';
            %     Tool = T_07;

            %     if sqrt(Tool(1,4)^2+Tool(2,4)^2)<514
            %         Err2 = 1;
            %         break
            %     end
            %
            handles = getappdata(0,'patch_h');           %
            L1 = handles(1);
            L2 = handles(2);
            L3 = handles(3);
            L4 = handles(4);
            L5 = handles(5);
            L6 = handles(6);
            L7 = handles(7);
            Tr = handles(9);
            %
            set(L1,'vertices',Link1(:,1:3),'facec', [0.717,0.116,0.123]);
            set(L1, 'EdgeColor','none');
            set(L2,'vertices',Link2(:,1:3),'facec', [0.216,1,.583]);
            set(L2, 'EdgeColor','none');
            set(L3,'vertices',Link3(:,1:3),'facec', [0.306,0.733,1]);
            set(L3, 'EdgeColor','none');
            set(L4,'vertices',Link4(:,1:3),'facec', [1,0.542,0.493]);
            set(L4, 'EdgeColor','none');
            set(L5,'vertices',Link5(:,1:3),'facec', [0.216,1,.583]);
            set(L5, 'EdgeColor','none');
            set(L6,'vertices',Link6(:,1:3),'facec', [1,1,0.255]);
            set(L6, 'EdgeColor','none');
            set(L7,'vertices',Link7(:,1:3),'facec', [0.306,0.733,1]);
            set(L7, 'EdgeColor','none');
            % store trail in appdata 
            if trail == 'y'
                x_trail = getappdata(0,'xtrail');
                y_trail = getappdata(0,'ytrail');
                z_trail = getappdata(0,'ztrail');
                %
                xdata = [x_trail T_04(1,4)];
                ydata = [y_trail T_04(2,4)];
                zdata = [z_trail T_04(3,4)];
                %
                setappdata(0,'xtrail',xdata); % used for trail tracking.
                setappdata(0,'ytrail',ydata); % used for trail tracking.
                setappdata(0,'ztrail',zdata); % used for trail tracking.
                %
                set(Tr,'xdata',xdata,'ydata',ydata,'zdata',zdata);
            end
            % Si la bolita está agarrada, actualizar su posición en la escena 3D con T_06
            if isappdata(0, 'ball_grasped') && getappdata(0, 'ball_grasped')
                b_h = getappdata(0, 'ball_handle');
                if ~isempty(b_h) && ishandle(b_h)
                    b_loc = getappdata(0, 'ball_local_pos');
                    if isempty(b_loc)
                        b_loc = [0; 0; 200; 1];
                    end
                    b_w = T_06 * b_loc;
                    ux = getappdata(0, 'sphere_unit_x');
                    uy = getappdata(0, 'sphere_unit_y');
                    uz = getappdata(0, 'sphere_unit_z');
                    if ~isempty(ux)
                        set(b_h, 'XData', ux + b_w(1), 'YData', uy + b_w(2), 'ZData', uz + b_w(3));
                    end
                end
            end
            drawnow
        end
        setappdata(0,'ThetaOld',[theta1,theta2,theta3,theta4,theta5,theta6]);
    end
%%
%
%
%%
    function InitHome
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI
        set(0,'Units','pixels')
        dim = get(0,'ScreenSize');
        fig_1 = figure('Position',[0,35,dim(3)-200,dim(4)-110],...
            'MenuBar','none','Name',' 3D Puma Robot Graphical Demo',...
            'NumberTitle','off','CloseRequestFcn',@del_app);
        hold on;
        %light('Position',[-1 0 0]);
        light                               % add a default light
        daspect([1 1 1])                    % Setting the aspect ratio
        view(135,25)
        xlabel('X'),ylabel('Y'),zlabel('Z');
        title('WWU Robotics Lab PUMA 762');
        axis([-1500 1500 -1500 1500 -1120 1500]);
        plot3([-1500,1500],[-1500,-1500],[-1120,-1120],'k')
        plot3([-1500,-1500],[-1500,1500],[-1120,-1120],'k')
        plot3([-1500,-1500],[-1500,-1500],[-1120,1500],'k')
        plot3([-1500,-1500],[1500,1500],[-1120,1500],'k')
        plot3([-1500,1500],[-1500,-1500],[1500,1500],'k')
        plot3([-1500,-1500],[-1500,1500],[1500,1500],'k')

        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        s5 = getappdata(0,'Link5_data');
        s6 = getappdata(0,'Link6_data');
        s7 = getappdata(0,'Link7_data');
        A1 = getappdata(0,'Area_data');
        %
        a2 = 650;
        a3 = 0;
        d3 = 190;
        d4 = 600;
        Px = 5000;
        Py = 5000;
        Pz = 5000;

        %The 'home' position, for init.
        t1 = 90;
        t2 = -90;
        t3 = -90;
        t4 = 0;
        t5 = 0;
        t6 = 0;
        
        % Forward Kinematics
        T_01 = tmat(0, 0, 0, t1);
        T_12 = tmat(-90, 0, 0, t2);
        T_23 = tmat(0, a2, d3, t3);
        T_34 = tmat(-90, a3, d4, t4);
        T_45 = tmat(90, 0, 0, t5);
        T_56 = tmat(-90, 0, 0, t6);

        % Each link fram to base frame transformation
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;
        
        % Actual vertex data of robot links
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';
        
        % points are no fun to watch, make it look 3d.
        L1 = patch('faces', s1.F1, 'vertices' ,Link1(:,1:3));
        L2 = patch('faces', s2.F2, 'vertices' ,Link2(:,1:3));
        L3 = patch('faces', s3.F3, 'vertices' ,Link3(:,1:3));
        L4 = patch('faces', s4.F4, 'vertices' ,Link4(:,1:3));
        L5 = patch('faces', s5.F5, 'vertices' ,Link5(:,1:3));
        L6 = patch('faces', s6.F6, 'vertices' ,Link6(:,1:3));
        L7 = patch('faces', s7.F7, 'vertices' ,Link7(:,1:3));
        A1 = patch('faces', A1.Fa, 'vertices' ,A1.Va(:,1:3));
        Tr = plot3(0,0,0,'b.'); % holder for trail paths
        %
        setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,A1,Tr])
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(L1, 'facec', [0.717,0.116,0.123]);
        set(L1, 'EdgeColor','none');
        set(L2, 'facec', [0.216,1,.583]);
        set(L2, 'EdgeColor','none');
        set(L3, 'facec', [0.306,0.733,1]);
        set(L3, 'EdgeColor','none');
        set(L4, 'facec', [1,0.542,0.493]);
        set(L4, 'EdgeColor','none');
        set(L5, 'facec', [0.216,1,.583]);
        set(L5, 'EdgeColor','none');
        set(L6, 'facec', [1,1,0.255]);
        set(L6, 'EdgeColor','none');
        set(L7, 'facec', [0.306,0.733,1]);
        set(L7, 'EdgeColor','none');
        set(A1, 'facec', [.8,.8,.8],'FaceAlpha',.25);
        set(A1, 'EdgeColor','none');
        %
        setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
        setappdata(0,'ball_grasped',0);
        %
    end
%%
    function T = tmat(alpha, a, d, theta)
        % tmat(alpha, a, d, theta) (T-Matrix used in Robotics)
        % The homogeneous transformation called the "T-MATRIX"
        % as used in the Kinematic Equations for robotic type
        % systems (or equivalent).
        %
        % This is equation 3.6 in Craig's "Introduction to Robotics."
        % alpha, a, d, theta are the Denavit-Hartenberg parameters.
        %
        % (NOTE: ALL ANGLES MUST BE IN DEGREES.)
        %
        alpha = alpha*pi/180;    %Note: alpha is in radians.
        theta = theta*pi/180;    %Note: theta is in radians.
        c = cos(theta);
        s = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        T = [c -s 0 a; s*ca c*ca -sa -sa*d; s*sa c*sa ca ca*d; 0 0 0 1];
    end
%%
    function del_app(varargin)
        %This is the main figure window close function, to remove any
        % app data that may be left due to using it for geometry.
        %CloseRequestFcn
        % here is the data to remove:
        %     Link1_data: [1x1 struct]
        %     Link2_data: [1x1 struct]
        %     Link3_data: [1x1 struct]
        %     Link4_data: [1x1 struct]
        %     Link5_data: [1x1 struct]
        %     Link6_data: [1x1 struct]
        %     Link7_data: [1x1 struct]
        %      Area_data: [1x1 struct]
        %        patch_h: [1x9 double]
        %       ThetaOld: [90 -182 -90 -106 80 106]
        %         xtrail: 0
        %         ytrail: 0
        %         ztrail: 0
        % Now remove them.
        rmappdata(0,'Link1_data');
        rmappdata(0,'Link2_data');
        rmappdata(0,'Link3_data');
        rmappdata(0,'Link4_data');
        rmappdata(0,'Link5_data');
        rmappdata(0,'Link6_data');
        rmappdata(0,'Link7_data');
        rmappdata(0,'ThetaOld');
        rmappdata(0,'Area_data');
        rmappdata(0,'patch_h');
        rmappdata(0,'xtrail');
        rmappdata(0,'ytrail');
        rmappdata(0,'ztrail');
        if isappdata(0, 'ball_handle')
            bh = getappdata(0, 'ball_handle');
            if ishandle(bh), delete(bh); end
            rmappdata(0, 'ball_handle');
        end
        if isappdata(0, 'ball_grasped'), rmappdata(0, 'ball_grasped'); end
        if isappdata(0, 'ball_local_pos'), rmappdata(0, 'ball_local_pos'); end
        if isappdata(0, 'ball_pos'), rmappdata(0, 'ball_pos'); end
        if isappdata(0, 'sphere_unit_x'), rmappdata(0, 'sphere_unit_x'); end
        if isappdata(0, 'sphere_unit_y'), rmappdata(0, 'sphere_unit_y'); end
        if isappdata(0, 'sphere_unit_z'), rmappdata(0, 'sphere_unit_z'); end
        if isappdata(0, 'traj_line_handle')
            tlh = getappdata(0, 'traj_line_handle');
            if ishandle(tlh), delete(tlh); end
            rmappdata(0, 'traj_line_handle');
        end
        if isappdata(0, 'last_traj_data'), rmappdata(0, 'last_traj_data'); end
        delete(fig_1);
    end
        
%%
    function [hout,ax_out] = uibutton(varargin)
        %uibutton: Create pushbutton with more flexible labeling than uicontrol.
        % Usage:
        %   uibutton accepts all the same arguments as uicontrol except for the
        %   following property changes:
        %
        %     Property      Values
        %     -----------   ------------------------------------------------------
        %     Style         'pushbutton', 'togglebutton' or 'text', default =
        %                   'pushbutton'.
        %     String        Same as for text() including cell array of strings and
        %                   TeX or LaTeX interpretation.
        %     Interpreter   'tex', 'latex' or 'none', default = default for text()
        %
        % Syntax:
        %   handle = uibutton('PropertyName',PropertyValue,...)
        %   handle = uibutton(parent,'PropertyName',PropertyValue,...)
        %   [text_obj,axes_handle] = uibutton('Style','text',...
        %       'PropertyName',PropertyValue,...)
        %
        % uibutton creates a temporary axes and text object containing the text to
        % be displayed, captures the axes as an image, deletes the axes and then
        % displays the image on the uicontrol.  The handle to the uicontrol is
        % returned.  If you pass in a handle to an existing uicontol as the first
        % argument then uibutton will use that uicontrol and not create a new one.
        %
        % If the Style is set to 'text' then the axes object is not deleted and the
        % text object handle is returned (as well as the handle to the axes in a
        % second output argument).
        %
        % See also UICONTROL.

        % Version: 1.6, 20 April 2006
        % Author:  Douglas M. Schwarz
        % Email:   dmschwarz=ieee*org, dmschwarz=urgrad*rochester*edu
        % Real_email = regexprep(Email,{'=','*'},{'@','.'})


        % Detect if first argument is a uicontrol handle.
        keep_handle = false;
        if nargin > 0
            h = varargin{1};
            if isscalar(h) && ishandle(h) && strcmp(get(h,'Type'),'uicontrol')
                keep_handle = true;
                varargin(1) = [];
            end
        end

        % Parse arguments looking for 'Interpreter' property.  If found, note its
        % value and then remove it from where it was found.
        interp_value = get(0,'DefaultTextInterpreter');
        arg = 1;
        remove = [];
        while arg <= length(varargin)
            v = varargin{arg};
            if isstruct(v)
                fn = fieldnames(v);
                for i = 1:length(fn)
                    if strncmpi(fn{i},'interpreter',length(fn{i}))
                        interp_value = v.(fn{i});
                        v = rmfield(v,fn{i});
                    end
                end
                varargin{arg} = v;
                arg = arg + 1;
            elseif ischar(v)
                if strncmpi(v,'interpreter',length(v))
                    interp_value = varargin{arg+1};
                    remove = [remove,arg,arg+1];
                end
                arg = arg + 2;
            elseif arg == 1 && isscalar(v) && ishandle(v) && ...
                    any(strcmp(get(h,'Type'),{'figure','uipanel'}))
                arg = arg + 1;
            else
                error('Invalid property or uicontrol parent.')
            end
        end
        varargin(remove) = [];

        % Create uicontrol, get its properties then hide it.
        if keep_handle
            set(h,varargin{:})
        else
            h = uicontrol(varargin{:});
        end
        s = get(h);
        if ~any(strcmp(s.Style,{'pushbutton','togglebutton','text'}))
            delete(h)
            error('''Style'' must be pushbutton, togglebutton or text.')
        end
        set(h,'Visible','off')

        % Create axes.
        parent = get(h,'Parent');
        ax = axes('Parent',parent,...
            'Units',s.Units,...
            'Position',s.Position,...
            'XTick',[],'YTick',[],...
            'XColor',s.BackgroundColor,...
            'YColor',s.BackgroundColor,...
            'Box','on',...
            'Color',s.BackgroundColor);
        % Adjust size of axes for best appearance.
        set(ax,'Units','pixels')
        pos = round(get(ax,'Position'));
        if strcmp(s.Style,'text')
            set(ax,'Position',pos + [0 1 -1 -1])
        else
            set(ax,'Position',pos + [4 4 -8 -8])
        end
        switch s.HorizontalAlignment
            case 'left'
                x = 0.0;
            case 'center'
                x = 0.5;
            case 'right'
                x = 1;
        end
        % Create text object.
        text_obj = text('Parent',ax,...
            'Position',[x,0.5],...
            'String',s.String,...
            'Interpreter',interp_value,...
            'HorizontalAlignment',s.HorizontalAlignment,...
            'VerticalAlignment','middle',...
            'FontName',s.FontName,...
            'FontSize',s.FontSize,...
            'FontAngle',s.FontAngle,...
            'FontWeight',s.FontWeight,...
            'Color',s.ForegroundColor);

        % If we are creating something that looks like a text uicontrol then we're
        % all done and we return the text object and axes handles rather than a
        % uicontrol handle.
        if strcmp(s.Style,'text')
            delete(h)
            if nargout
                hout = text_obj;
                ax_out = ax;
            end
            return
        end

        % Capture image of axes and then delete the axes.
        frame = getframe(ax);
        delete(ax)

        % Build RGB image, set background pixels to NaN and put it in 'CData' for
        % the uicontrol.
        if isempty(frame.colormap)
            rgb = frame.cdata;
        else
            rgb = reshape(frame.colormap(frame.cdata,:),[pos([4,3]),3]);
        end
        size_rgb = size(rgb);
        rgb = double(rgb)/255;
        back = repmat(permute(s.BackgroundColor,[1 3 2]),size_rgb(1:2));
        isback = all(rgb == back,3);
        rgb(repmat(isback,[1 1 3])) = NaN;
        set(h,'CData',rgb,'String','','Visible',s.Visible)

        % Assign output argument if necessary.
        if nargout
            hout = h;
        end
%%
    end
end
% Finally.