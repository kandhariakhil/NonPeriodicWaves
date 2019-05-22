%Function calculates trapezoid corners based on input side lengths and plots the final result

function [A,o,B] = CalculateTrapezoidSegments(w_left, w_right, d, printout, posteriorcenters, plotcolor, plotthickness)

if nargin <7
    plotthickness = 1;
end
if nargin <6
    plotcolor = [0,0,0];
end
    
if nargin<5 || isempty(posteriorcenters)
    o(1,:) = [0,0];
    A(1,:) = [0,d/2];
    B(1,:) = [0,-d/2];
    startangle = 0;
else

    o(1,:) = posteriorcenters(2,:);
    A(1,:) = posteriorcenters(1,:);
    B(1,:) = posteriorcenters(3,:);
    startangle= atan2(posteriorcenters(1,1)- posteriorcenters(3,1), ... 
                      posteriorcenters(1,2)- posteriorcenters(3,2));
    
end

number_segments = length(w_left);

beta = acos((w_right - w_left)/(2*d));
w_center = w_left+d*cos(beta);
phi(1) = pi/2-beta(1)-startangle;
phi(2:number_segments) = pi-beta(1:end-1)-beta(2:end);
totalsegmentrotate = cumsum(phi);

for i = 1:number_segments 
 forwardunitvector = [cos(totalsegmentrotate(i)), sin(totalsegmentrotate(i))];   
 A(i+1,:) = A(i,:)+ w_left(i)*forwardunitvector;
 o(i+1,:) = o(i,:) + w_center(i)*forwardunitvector;
 B(i+1,:) = B(i,:) + w_right(i)*forwardunitvector;   
end

if printout

 for i = 1:number_segments
     plotsegment = [A(i,:); A(i+1,:);B(i+1, :); B(i,:); A(i,:) ]  ; 
     plot(plotsegment(:,1), plotsegment(:,2), '-k.', 'Color', plotcolor, 'LineWidth', plotthickness)
          hold on    
 end
 
 axis equal
 
end
