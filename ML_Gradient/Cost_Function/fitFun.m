function P = fitFun(dF,IG)
%FITFUN Summary of this function goes here
%   Detailed explanation goes here
xfitted=dF(1,:);
yfitted=dF(2,:);

    function [ds]=fit(IG)
       ato=IG(1); 
       bto=IG(2); 
       cto=IG(3); 
       dto=IG(4);
       
       f=ato.*exp(bto.*xfitted+cto)+dto;
       
       ds=(f-yfitted).^2;
       ds=sum(ds);
    end
P=fminsearch(@fit,IG);
end