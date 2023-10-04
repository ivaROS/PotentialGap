function J = pgCost(X,U,e,data,params)
    J = sum(X(1:ddTraj.minst.PredictionHorizon, 1:2) * normal_1') + sum(X(1:ddTraj.minst.PredictionHorizon, 1:2) * normal_2');
end