function dydt = diffFunc(t,y,p)
dydt = [y(2); sin(y(1) + p(1))];
end