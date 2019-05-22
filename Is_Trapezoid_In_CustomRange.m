%Define bounded region of segment and checking if new found lengths of trapezoids is within given
%region

function inbounds = Is_Trapezoid_In_CustomRange(wl, wr, lowerboundpoints, showplot) 

wlong = max([wl; wr]);
wshort = min([wl; wr]);

inbounds = (wshort >= min(min(lowerboundpoints))) ...
    & ( wlong <= max(max(lowerboundpoints)));

inbounds = inbounds & wshort >= interp1( lowerboundpoints(:,1),lowerboundpoints(:,2), wlong); 
    
if showplot
    figure(12)
    clf(12)
    figure(12)
    hold on
    plot(lowerboundpoints(:,1), lowerboundpoints(:,1), 'k-');
    plot(lowerboundpoints(:,2), lowerboundpoints(:,1), 'b:');
    plot(lowerboundpoints(:,1), lowerboundpoints(:,2), '-bo')
    plot( wl(inbounds), wr(inbounds), 'b*')
    plot( wl(~inbounds), wr(~inbounds), 'r*')
end


