function [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,sensor_model,R,Q)
%[returns vars] = fnName(input vars)
    % Prediction
    mup = g;
    n=length(mup);
    Sp = Gt*S*Gt' + R;
    
    % update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    disp(Y-sensor_model(mup));
    mu = mup+ K*(Y-sensor_model(mup));
    S = (eye(n)-K*Ht)*Sp;
end