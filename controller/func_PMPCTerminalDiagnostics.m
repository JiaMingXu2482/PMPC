function d=func_PMPCTerminalDiagnostics(cert) %#codegen
% Explicit fixed-size PMPC output contract, independent of other variants.
d=zeros(20,1);
d(1:12)=func_PriorityDiagnostics(cert(1:36));
d(13:20)=cert(37:44);
end
