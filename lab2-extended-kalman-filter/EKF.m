function [mu,S,K,mup] = EKF(f,g,Gt,Ht,S,Y,sensor_model,R,Q)
%[returns vars] = fnName(input vars)
    % Prediction
    mup = g;
    n=length(mup);
    Sp = Gt*S*Gt' + R;
    
    % update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup+ K*(Y-sensor_model(f));
    S = (eye(n)-K*Ht)*Sp;
end