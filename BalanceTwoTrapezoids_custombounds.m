%Code written by Kathryn Daltorio(kam37@case.edu) and Akhil Kandhari(axk751@case.edu)
%Results of this code have been published in Soft Robotics:
%Kandhari, A., Wang, Y., Chiel, H.J. and Daltorio, K.A., 2019. Turning in Worm-Like Robots: The Geometry of Slip Elimination Suggests Nonperiodic Waves. Soft robotics.
%https://www.liebertpub.com/doi/full/10.1089/soro.2018.0080

%Code checks balancing of two trapezoid during turning while keeping no slip condition
%Follows kinematic constraint equations to prevent slip

%Left and right lenghts of trapezoid calculated using kinematic constraint equations
function [w_left, w_right] = BalanceTwoTrapezoids_custombounds(frontindex, w_left_original, w_right_original, d, plotme, lowerboundpoints)

w_left = []; 
w_right = [];
new_w_left = w_left_original;
new_w_right = w_right_original;

secondindex = frontindex -1;
PreventLeftToRightSway = false && (new_w_left(secondindex) ~= new_w_right(secondindex));

LeftShortOriginal = w_left_original(secondindex) < w_right_original(secondindex);
RightShortOriginal = w_left_original(secondindex) > w_right_original(secondindex);

%Calculate constants
beta = acos((w_right_original - w_left_original)/(2*d));
capphi = beta(secondindex)+beta(frontindex); %Final rotation to be constant
w_center = w_left_original+d*cos(beta);
s = sqrt( w_center(secondindex)^2+ w_center(frontindex)^2 - 2*w_center(secondindex)*w_center(frontindex)*cos(capphi)); %Length between two trapezoid edges to be constant
original_w_center = w_center;
inc_range = max(max(lowerboundpoints))-w_center(secondindex);
inc_res = inc_range/50;
cumincrease = 0;
check = true;
i = 0;

while all(Is_Trapezoid_In_CustomRange(new_w_left, new_w_right, lowerboundpoints, false)) && check
    i = i +1;
    w_left = new_w_left; %save working trapezoids in case next step out of bounds
    w_right = new_w_right;
    cumincrease = cumincrease + inc_res;
    
    wc1 = original_w_center(secondindex) + cumincrease; %Increase center of trapezoid
    wc2 = (2*wc1*cos(capphi) + sqrt(4*(s^2 - wc1^2*sin(capphi)^2)))/2; %Solve for second center with quadratic formula based on constraint equation
    originalgam = asin(sin(capphi)/s * w_center(frontindex));
    newgam = asin(sin(capphi)/s * wc2); %Using Sin rule
    newt1 = beta(secondindex) - originalgam + newgam;
    new_w_left(secondindex) =  wc1 - d* cos(newt1);
    new_w_right(secondindex) = new_w_left(secondindex)+2*d*cos(newt1);
    new_w_left(frontindex) =  wc2 - d* cos(capphi-newt1);
    new_w_right(frontindex) = new_w_left(frontindex)+2*d*cos(capphi-newt1); 
    
    if PreventLeftToRightSway
        check  = LeftShortOriginal*(new_w_left(secondindex) < new_w_right(secondindex)) ...
                + RightShortOriginal*(new_w_left(secondindex) > new_w_right(secondindex)) ...
                +(new_w_left(secondindex) == new_w_right(secondindex));
    end
    
    if plotme
        figure(2)
        hold on
        plot(new_w_left(secondindex), new_w_right(secondindex), '.b')
        plot(new_w_left(frontindex), new_w_right(frontindex), '.r')
        
        figure(1)
        if mod(i,4) == 0 
        hold on
        CalculateTrapezoidSegments(new_w_left, new_w_right, d, plotme, [],[.5, .5, .5]);
        end 
        
        figure(3)
        hold on 
        plot(i, .5*(new_w_left(secondindex) + new_w_right(secondindex)) - d, '.b-')
        plot(i, new_w_right(secondindex) - new_w_left(secondindex), ':bo')
        
        plot(i, .5*(new_w_left(frontindex) + new_w_right(frontindex)) - d, '.r-')
        plot(i, new_w_right(frontindex) - new_w_left(frontindex), ':ro')
        
    end
end
    

