%Code written by Kathryn Daltorio(kam37@case.edu) and Akhil Kandhari(axk751@case.edu) 
%Results of this code have been published in Soft Robotics:
%Kandhari, A., Wang, Y., Chiel, H.J. and Daltorio, K.A., 2019. Turning in Worm-Like Robots: The Geometry of Slip Elimination Suggests Nonperiodic Waves. Soft robotics.
%https://www.liebertpub.com/doi/full/10.1089/soro.2018.0080

%The following kinematic model is based on Compliant Modular Mesh worm Robot with Steering
%https://iopscience.iop.org/article/10.1088/1748-3190/aaa342/meta

%Main code to run non periodic waveform for tunring without slip in peristaltic locomotion
%Code produces waveform required to reach certain end position in order to turn without slip using
%kinematic constraints

clear all;
makemovie = true;
angleindegrees = 90; %Angle by which robot needs to turn
num_waves = 25; %Number of waves

%The following bounded points refers to the reachable configurations of a single segment
%Data taken from CMMWorm-S exaggarated version for simulation purposes
lowerboundpoints = [21.75, 21.75;
                     23.6, 20.9; 
                     26, 20.5;
                     36, 18];

%Accurate lowerboudpoints based on robot segment:
lowerboundpoints = lowerboundpoints/1.8643;

d = 20; %nominal (max) diameter of segments (trapezoid height)
number_segments = 6; %Number of segments in CMMWorm-S
min_w = lowerboundpoints(1,1); %Minimum length of a segment
max_w = lowerboundpoints(end,1); %Maximum length of a segment
minw_at_maxw = lowerboundpoints(end,2); %Minimum length at maximum length (limit of segment extension) 

axislimits = [-20,200,-50,150];

%Setting points forming trapezoid edges for plotting purposes
firstw = lowerboundpoints(1,1);
w_left = firstw*ones(1,number_segments);
w_right = w_left;

if makemovie
    WormMovie = VideoWriter('Demo_movie');
    open(WormMovie);
    figure(1)
    clf(1)
    figure(1)
end

%Based on left segment lengths, right segment lengths and diameter of trapezoids, calculate trapezoid corners.
[A,o,B] = CalculateTrapezoidSegments(w_left, w_right, d, makemovie);

if makemovie
    axis(axislimits)
    writeVideo(WormMovie, getframe(gca));
end

%Last segment points from which all corner points are calculated for each trapezoid
backpoints = [A(1,:); o(1,:); B(1,:)];

N = length (w_left);
all_w_left = [firstw*ones(1,N); w_left];
all_w_right = [firstw*ones(1,N); w_right];
frontpoints = [];
allpoints_at_wave = [];
trailplotcolors = cool(N+1); %Color coding for ploting each segments path

for waves = 1:num_waves
    frontpoints(waves,:) = o(end,:);
    allpoints_at_wave(:, :, waves) = o;
    
    %Finding first segment reach for each wave
    alpha = atan2(A(end-2,2)-B(end-2,2),-(A(end-2,1)-B(end-2,1))); %Starting angle of the segment to expand
    beta = pi-alpha-(angleindegrees*pi/180); % Desired angle of the segment to be expanded
    differenceinsidelengths = 2*cos(beta/2)*d; %Difference between left and right side length of a segment
    
    if differenceinsidelengths>0
        w_right(end-1) = max_w;
        w_left(end-1) = max(max_w-differenceinsidelengths, firstw);
    else
        w_left(end-1) = max_w;
        w_right(end-1) = max(max_w-abs(differenceinsidelengths), firstw);
    end
    
    CalculateTrapezoidSegments(w_left, w_right, d, 1, backpoints);
    
    if makemovie
        clf(1)
    end
    
    CalculateTrapezoidSegments(w_left, w_right, d, makemovie, backpoints);
    
    if  makemovie
        text(0,axislimits(1)*.8, num2str(waves))
        axis(axislimits)
        writeVideo(WormMovie, getframe(gca));
    end
    
    %Updating left and right lengths
    all_w_left = [all_w_left;w_left];
    all_w_right = [all_w_right;w_right];
    
    %Calculating trapezoid corners using bounded points as reference
    for forwardanchor = N:-1: 4
        [new_w_left, new_w_right] = BalanceTwoTrapezoids_custombounds(forwardanchor-1, w_left, w_right, d, 0, lowerboundpoints);
        figure(1) 
        
        w_left = new_w_left;
        w_right = new_w_right;
        all_w_left = [all_w_left;w_left];
        all_w_right = [all_w_right;w_right];
        
        if makemovie
            clf(1)
        end
        
        [A,o,B] = CalculateTrapezoidSegments(w_left, w_right, d, makemovie, backpoints);
        
        if makemovie
            text(0,axislimits(1)*.8, num2str(waves))
            axis(axislimits)
            writeVideo(WormMovie, getframe(gca));
        end
    end
    
    %Last but one segment's edges
    thirda = A(3,:);
    thirdo = o(3,:);
    thirdb =  B(3,:);
    w_right(1:2) = w_left(1);
    w_left(1:2) = w_left(1);
    
    %This dictates overall angle of the robot, i.e., orientation of the robot
    anglenew = atan2(thirda(2)-thirdb(2),thirda(1)-thirdb(1));
    backpoints = d*.5*[cos(anglenew),sin(anglenew);
        0,0;
        -cos(anglenew),-sin(anglenew)];
    
    %Moving of last segment to control total forward motion such taht ultimate segment follows
    %penultimate segment with constant contraction and expansion cycles.
    [unshiftedA,unshiftedo,unshiftedB] = CalculateTrapezoidSegments(w_left, w_right, d, 0, backpoints);
    
    backpoints = d*.5*[cos(anglenew),sin(anglenew);
        0,0;
        -cos(anglenew),-sin(anglenew)]+[1;1;1]*(thirdo-unshiftedo(3,:));
    
    clf(1)
    figure(1)
    [shiftedA,shiftedo,shiftedB] = CalculateTrapezoidSegments(w_left, w_right, d, 1, backpoints);
    text(0,axislimits(1)*.8, num2str(waves))
    axis(axislimits)
    
    if makemovie
        writeVideo(WormMovie, getframe(gca));
    end
    
    shiftposition_error = sqrt((thirda-shiftedA(3,:)).^2+(thirdo-shiftedo(3,:)).^2+(thirdb-shiftedB(3,:)).^2);
    
    if shiftposition_error >10^-10
        error('high shift position error!');
    end
    
    all_w_left = [all_w_left;w_left];
    all_w_right = [all_w_right;w_right];
    
end

%Plot trajectory of each segment
figure(2)
hold on
for plotseg = 1:(N+1)
    plot(squeeze(allpoints_at_wave(plotseg,1,:)), squeeze(allpoints_at_wave(plotseg,2,:)), '.b-', 'Color', trailplotcolors(plotseg,:))
end

if makemovie
    close(WormMovie)
end

%Plot control method for each segment
figure(3)
clf(3)
figure(3)
hold on
extension = (.5*(all_w_right + all_w_left) - firstw)/firstw;
bias = (all_w_right - all_w_left)/firstw;
plotcolors = [cool(7); cool(7);cool(7)];
spacing = 1;

for seg = N:-1:N-5
    plot(extension(:,seg)+(spacing)*(seg-1), '-k.');
    plot(bias(:,seg)+(spacing)*(seg-1), ':ro', 'Color', plotcolors(seg,:))
    plot([0, length(bias)],[1,1]*(spacing)*(seg-1), 'r-', 'Color', plotcolors(seg,:))
    text(length(bias)*1.01, (spacing)*(seg-1), ['Segment ', num2str(N-seg)], 'Color',  plotcolors(seg,:))
end

legend('Center length', 'bias')
xlabel('timesteps')
ylabel('control input (stacked)')

%Plot extension of each segment
figure(4)
clf(4)
figure(4)
hold on
for seg = N:-1:1
    plot(extension(:,seg)-firstw, '-k.', 'Color', plotcolors(seg,:))
end
