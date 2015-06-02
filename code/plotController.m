function plotController(C)
%
% This function plots the results of running the controller design process.
% For each target speed, it plots push-off impulse, step angle, and the
% one-step map.
%

nTarget = length(C.wTarget);    
    
clf;
for i=1:nTarget
    
    idx_1 = 1 + (i-1)*3;
    idx_2 = 2 + (i-1)*3;
    idx_3 = 3 + (i-1)*3;
    
    idxFail = C.exitFlag(:,i)~=1;
    
    %Push-Off
    subplot(nTarget,3,idx_1);hold on
    plot(C.wMeasured, C.p(:,i))
    ylabel('push-off');
    title(['target speed: ' num2str(C.wTarget(i))]);
    if i==nTarget, xlabel('measured speed'); end
    plotTargetSpeed(C.wTarget(i), C.p(:,i));
    plot(C.wMeasured(idxFail), C.p(idxFail,i),'rx')
    axis tight
    
    %%% Step Angle
    subplot(nTarget,3,idx_2); hold on;
    plot(C.wMeasured, C.phi(:,i))
    ylabel('Step Angle');
    title(['target speed: ' num2str(C.wTarget(i))]);
    if i==nTarget, xlabel('measured speed'); end
    plotTargetSpeed(C.wTarget(i), C.phi(:,i));
    plot(C.wMeasured(idxFail), C.phi(idxFail,i),'rx')
    axis tight

    %%% Plot the one-step map
    subplot(nTarget,3,idx_3); hold on
    err = C.wMeasured - C.wTarget(i);
    plot(C.wMeasured, C.wTarget(i) + abs(err),'k--');
    plot(C.wMeasured, C.wTarget(i) - abs(err),'k--');
    for j=1:16
        plot(C.wMeasured, C.wFinalRobust(:,i,j) ,'r-','LineWidth',1)
    end
    plot(C.wMeasured, C.wFinal(:,i))
    ylabel('Final Speed Error');
    title(['target speed: ' num2str(C.wTarget(i))]);
    if i==nTarget, xlabel('Initial Speed Error'); end
    if isfield(C,'wFinalMax')
        plot(C.wMeasured, C.wFinalMax(:,i),'g--');
        plot(C.wMeasured, C.wFinalMin(:,i),'g--');
    end
    axis tight
    
end

end



function plotTargetSpeed(xTarget,y)

xx = xTarget*[1,1];
yy = [min(y),max(y)];
plot(xx,yy,'k--','LineWidth',2);

end


