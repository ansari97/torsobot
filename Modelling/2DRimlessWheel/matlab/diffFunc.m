function dydt = diffFunc(t,y,p)
dydt = [y(2); 1/p(end)*(sin(y(1) + p(1)))];
end