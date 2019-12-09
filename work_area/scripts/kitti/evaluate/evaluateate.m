function [RMSE,Mean,Median,Std,Min,Max] = evaluateate(s)
a = length(s);
for i = 1:a
    ate(i) = sqrt((s(i,1)-s(i,3))^2+(s(i,2)-s(i,4))^2);
end
Mean = mean(ate);
RMSE = sqrt(sum((ate-Mean).^2)/a);
Median = median(ate);
Std = std(ate);
Min = min(ate(2:a));
Max = max(ate);
end