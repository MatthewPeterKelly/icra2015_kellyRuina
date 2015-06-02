function saveResults(C,R)

save('RobustController.mat','C','R');

plotArea = [600,600];  %Area in pixels of the plot

LINE_WIDTH_HEAVY = 3;
LINE_WIDTH_LIGHT = 2;

LINE_COLOR_MAIN = [0.1, 0.1, 0.9];
LINE_COLOR_DASHED = 0.1*[1,1,1];
LINE_COLOR_OTHER = [0.9, 0.1, 0.1];

%How much to pad the axis of each subplot with white space
rx = 1.05;
ry = 1.05;

%%% Shared calculations:
nTarget = R.config.nTargetSpeeds;
bndSpeed = [R.measuredSpeed(1),R.measuredSpeed(end)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Push-Off Controller                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure(1425); clf;
setFigureArea(plotArea)
bndPush = [0,1];
for i=1:nTarget
    subplot(nTarget,1,i); hold on
    plot(R.measuredSpeed, R.control(i).pushOff,...
        'LineWidth',LINE_WIDTH_HEAVY,...
        'Color',LINE_COLOR_MAIN);
    plot(R.speed(i).target*[1,1],...
        bndPush,...
        '--','LineWidth',LINE_WIDTH_LIGHT,...
        'color',LINE_COLOR_DASHED);
    axis(padAxis([bndSpeed, bndPush],rx,ry));
end
c1 = get(f1,'Children');
for i=1:3
    set(c1(i),'XTick',linspace(0,1,5));
end
save2pdf('Fig_Raw_pushOff.pdf',gcf);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Step Angle Controller                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f2 = figure(1426); clf;
setFigureArea(plotArea)
bndStep = [0, 1];
for i=1:nTarget
    subplot(nTarget,1,i); hold on
    plot(R.measuredSpeed, R.control(i).stepAngle,...
        'LineWidth',LINE_WIDTH_HEAVY,...
        'Color',LINE_COLOR_MAIN);
    plot(R.speed(i).target*[1,1],...
        bndStep,...
        '--','LineWidth',LINE_WIDTH_LIGHT,...
        'color',LINE_COLOR_DASHED);
    axis(padAxis([bndSpeed, bndStep],rx,ry));
end
c2 = get(f2,'Children');
for i=1:3
    set(c2(i),'XTick',linspace(0,1,5));
end
save2pdf('Fig_Raw_stepAngle.pdf',gcf);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            One Step Map                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f3 = figure(1427); clf; hold on;
setFigureArea(plotArea)
i = 2;  %Show the second target speed for now;

wTarget = R.speed(i).target;

%%% Plot the "X" for identity step map
err = R.measuredSpeed - R.speed(i).target;
target = R.speed(i).target;
plot(err+target, [err, -err]+target,...
    '--','LineWidth',LINE_WIDTH_LIGHT,...
    'color',LINE_COLOR_DASHED);

%%% Plot the nominal step map
plot(err+target, R.speed(i).nom-wTarget+target,...
    'LineWidth',LINE_WIDTH_HEAVY,...
    'Color',LINE_COLOR_MAIN);

%%% Plot the limits on the step map
plot(err+target, R.speed(i).min-wTarget+target,...
    'LineWidth',LINE_WIDTH_HEAVY,...
    'Color',LINE_COLOR_OTHER);
plot(err+target, R.speed(i).max-wTarget+target,...
    'LineWidth',LINE_WIDTH_HEAVY,...
    'Color',LINE_COLOR_OTHER);
axis equal;
axis([0,1,0,1]); 

c3 = get(f3,'Children');
set(c3(1),'XTick',linspace(0,1,5))
set(c3(1),'YTick',linspace(0,1,5))
save2pdf('Fig_Raw_stepMap.pdf',gcf);

end


function setFigureArea(plotArea)
position = get(gcf,'Position');
position(3:4) = plotArea;
set(gcf,'Position',position);
end


function out = padAxis(in, rx,ry)

xIn = in(1:2);
yIn = in(3:4);
center = [mean(xIn), mean(yIn)];

xInShift = xIn - center(1);
yInShift = yIn - center(2);

xScale = rx*xInShift;
yScale = ry*yInShift;

out = [xScale + center(1), yScale + center(2)];

end