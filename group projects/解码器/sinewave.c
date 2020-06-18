//////////////////////////////////////////////////////////////////////// 
//  generates a sinusoid of dwLength in dwData 
//      and maintains phis value updated
////////////////////////////////////////////////////////////////////////
function [dwDataOut,phis] = rc(dwLength, phis)
omegaf=7000/dwLength/%pi*90/100;

dwDataOut = zeros(1,dwLength);

for dwi=1:dwLength,
    dwDataOut(dwi) = 4450*sin(dwi*omegaf+phis)+200;
end

//update wave phase
//WAS: phis = phis - %pi*(1-50/dwLength)+%pi/2;
phis = dwLength*omegaf+phis;
while(phis<0)
    phis = phis + 2*%pi;
end
endfunction