function J = CustomCostFcn(X,U,e,data,params)
    L = params(1);
    step_time = params(2);
    initial_speed = params(3);
    J = sum((X(2:end,1) - data.References(:,1)).^2 + (X(2:end,2) - data.References(:,2)).^2 + (initial_speed*step_time*(X(2:end,3) - data.References(:,3))).^2 + ((initial_speed*step_time)^2/L*(X(2:end,4) - data.References(:,4))).^2);
