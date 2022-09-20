function [s] = Kalman_Filter(s)
   %Prediction Phase
    %prediction of state X
     X_priori = s.A * s.x + s.B * s.u;
    %prediction of error covariance P 
     P_priori = s.A * s.P * transpose(s.A) + s.Q;
   %Kalman Gain Calculation 
     Kalman_Gain = P_priori * transpose(s.H) * (s.H * P_priori * transpose(s.H) + s.R)^(-1);
   %Calibration Phase 
    %Calibration of state X  
     s.x = X_priori + Kalman_Gain * (s.z - (s.H * X_priori));
     %Calibration of covariance P 
     s.P = (eye(5) - Kalman_Gain * s.H) * P_priori;   
    
     %Overall steps based on L8, slide 11, CPE 470 
    
end
