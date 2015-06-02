function C = getStabilityRegion(C)
%
% This function computes the regions where the controller is
% unconditionally stable.
%
%

nTarget = length(C.wTarget);
w = C.wMeasured;
index = 1:length(w);
for i=1:nTarget
    wTarget = C.wTarget(i);
    wMin =C.wFinalMin(:,i);
    wMax =C.wFinalMax(:,i);
    
    errInit = (w - wTarget).^2;
    errFinal = max((wMin - wTarget).^2,(wMax - wTarget).^2);
    lyap = errFinal - errInit;
    
    stable = errFinal < errInit;
    if sum(~stable)==0
        C.stability(i).wLow = min(C.wMeasured);
        C.stability(i).wTarget = wTarget;
        C.stability(i).wUpp = max(C.wMeasured);
    else
        idxLow = index(diff(stable)==-1); idxLow = idxLow + [0,1];
        idxUpp = index(diff(stable)==1); idxUpp = idxUpp + [0,1];
        
        wLow = interp1(lyap(idxLow), w(idxLow), 0);  %Linear root finding
        wUpp = interp1(lyap(idxUpp), w(idxUpp), 0);  %Linear root finding
        
        C.stability(i).wLow = wLow;
        C.stability(i).wTarget = wTarget;
        C.stability(i).wUpp = wUpp;
    end
    
end

end
