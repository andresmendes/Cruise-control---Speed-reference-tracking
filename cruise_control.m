%% Cruise control - Speed reference tracking
% Simulation and animation of a vehicle with cruise control and varying
% speed reference.
%%

clear ; close all ; clc

%% Scenario

% Road
road_Width              = 10;       % Road width                    [m]
road_Margin             = 2;        % Road margin                   [m]
road_Dist_Analysis      = 100;      % Road distance analysis        [m]
% Vehicle
vehicle_Length          = 4.65;     % Length of the vehicle         [m]
vehicle_Width           = 1.78;     % Width of the vehicle          [m]
vehicle_Initial_Speed   = 72/3.6;   % Initial speed of the vehicle  [m/s]
vehicle_Mass            = 1000;     % Mass of the vehicle           [kg]
vehicle_Area            = 2.5;      % Frontal area of the vehicle   [m2]
vehicle_Cd              = 0.35;     % Drag coefficient              [-]
air_Density             = 1;        % Air density                   [kg/m3]

% Lumped air drag coefficient [N(s/m)2]
C = 0.5 * vehicle_Area * vehicle_Cd * air_Density;   

% Vehicle struct
vehicle.C = C;
vehicle.M = vehicle_Mass;

% Parameters
tf      = 60;                       % Final time                    [s]
fR      = 30;                       % Frame rate                    [fps]
dt      = 1/fR;                     % Time resolution               [s]
TSPAN   = linspace(0,tf,tf*fR);     % Time                          [s]

%% Simulation

% Initial conditions [position speed]
Z0 = [0 vehicle_Initial_Speed]; 

% Integration
options = odeset('RelTol',1e-6);
[TOUT,ZOUT] = ode45(@(t,z) vehicle_dynamics(t,z,vehicle),TSPAN,Z0,options);

% States
vehicle_position    = ZOUT(:,1);
vehicle_speed       = ZOUT(:,2);
% Acceleration
% Preallocating
vehicle_acc         = zeros(1,length(TOUT));
force_long          = zeros(1,length(TOUT));
speed_ref           = zeros(1,length(TOUT));
for i=1:length(TOUT)
    [dz,F_l,V_r]    = vehicle_dynamics(TOUT(i),ZOUT(i,:),vehicle);
    vehicle_acc(i)  = dz(2);
    force_long(i)   = F_l;
    speed_ref(i)    = V_r;
end

%% Results

figure
set(gcf,'Position',[270   140   640     360  ])

% Create and open video writer object
v = VideoWriter('cruise_control.avi');
v.Quality = 100;
open(v);

for i=1:length(TOUT)
    subplot(3,2,1)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*max(vehicle_position)])
        cla 
        plot(TOUT,vehicle_position,'b')
        plot([TOUT(i) TOUT(i)],[0 1.2*max(vehicle_position)],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
    subplot(3,2,2)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*max(vehicle_speed)])
        cla 
        plot(TOUT,speed_ref,'k')
        plot(TOUT,vehicle_speed,'b')
        plot([TOUT(i) TOUT(i)],[0 1.2*max(vehicle_speed)],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed (Black=Reference, Blue=Actual)')
    subplot(3,2,3)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*min(vehicle_acc) 1.2*max(vehicle_acc)])
        cla 
        plot(TOUT,vehicle_acc,'b')
        plot([TOUT(i) TOUT(i)],[1.2*min(vehicle_acc) 1.2*max(vehicle_acc)],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
    subplot(3,2,4)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*min(force_long) 1.2*max(force_long)])
        cla 
        plot(TSPAN,force_long,'b')
        plot([TOUT(i) TOUT(i)],[1.2*min(force_long) 1.2*max(force_long)],'k--') 
        xlabel('Time [s]')
        ylabel('Lon. force [N]')
        title('Longitudinal force')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % Position of the vehicle at current instant [m]
        vehicle_position_inst = vehicle_position(i);

        sideMarkingsX = [vehicle_position_inst-road_Dist_Analysis/2 vehicle_position_inst+road_Dist_Analysis/2];
        set(gca,'xlim',sideMarkingsX,'ylim',[-road_Width/2-road_Margin +road_Width/2+road_Margin])

        plot(sideMarkingsX,[+road_Width/2 +road_Width/2],'k--') % Left marking
        plot(sideMarkingsX,[-road_Width/2 -road_Width/2],'k--') % Right marking

        % Dimensions
        vehicle_dimension_X = [vehicle_position_inst vehicle_position_inst vehicle_position_inst-vehicle_Length vehicle_position_inst-vehicle_Length];
        vehicle_dimension_Y = [+vehicle_Width/2 -vehicle_Width/2 -vehicle_Width/2 +vehicle_Width/2];
        % Plotting
        fill(vehicle_dimension_X,vehicle_dimension_Y,'r')
        
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

%% Auxiliary functions

function [dstates,F_lon,V_ref] = vehicle_dynamics(t,states,vehicle)
    
    % Parameters
    m = vehicle.M;              % Mass of the vehicle           [kg]
    C = vehicle.C;              % Lumped air drag coefficient   [N(s/m)2]

    % States
%     X = states(1);
    V = states(2);

    % Drag resistance
    Dx = C*V^2;
    
    % Reference speed [m/s]
    if t < 20
        V_ref   = 25; 
    elseif t < 40
        V_ref   = 10; 
    else
        V_ref   = 20;
    end
    
    % Cruise controller
    Kp = 500;
    F_lon  = Kp*(V_ref - V) + Dx;
    
    % Dynamics
    dstates(1,1) = V;
    dstates(2,1) = (F_lon - Dx)/m;
    
end
