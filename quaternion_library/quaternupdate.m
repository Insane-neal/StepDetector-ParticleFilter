function q = quarternupdate(w,qi)
%ROTMAT2QUATERN Converts a rotation matrix orientation to a quaternion
%
%   q = quarternionupdate(w, q)
%
%   update the quarternion based on gyro output
%
%	Date          Author          Notes
%	06/22/2016     JL          


w1 = w(1);
w2 = w(2);
w3 = w(3);

W = [ 0 -w1 -w2 -w3; ...
      w1 0  -w3  w2; ...
      w2 w3  0   -w1;...
      w3 -w2 w1   0]; 
  
mag_w = sqrt(w1^2+w2^2+w3^2); 
I     = eye(4); 

q = (cos(mag_w)*I + (sin(mag_w)/mag_w)*W)*qi'; 
q = q'; 

  