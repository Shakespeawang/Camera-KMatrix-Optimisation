function [T_cw] = PositionCameraForGrid(T_ow,Distance)
%PositionCamera
% Generates a 'random' camera frame that almost points at the origin of the
% calibration object back along the object's z-axis (which is perturbed). Aim
% is to 'fill' the camera image with calibration grid
%
% T_ow is the 4x4 transformation matrix that places the calibration grid
% (normally centred on origin, in local x-y plane only). Distance is the
% distance from camera to grid origin along normal

% Get object origin from T_ow
ObjectOrigin = T_ow(1:3,4);

% View the grid from 'Distance' along the grid's negative world z-axis (normal),
% with a small amount of randomness
ViewVector = -Distance*T_ow(1:3,3) + 0.1*Distance*rand(3,1);

% Create camera origin in world coordinates from this
CameraOrigin = ObjectOrigin - ViewVector;

% Want to align the camera z axis with ViewVector.
Cameraz = ViewVector./norm(ViewVector);  % Normalised camera z axis

% Perturb the camera z axis slightly to add some angle to the viewing of grid
Cameraz = Cameraz - 0.01*(rand(3,1)-0.5); % -0.5 shifts rand to 0 mean
% Normalise again
Cameraz = Cameraz / norm(Cameraz);

% Generate orthogonal (to z and each other) but otherwise random unit x, y camera axes
ynorm = 0;
while ynorm < eps
    y = rand(3,1);
    y = y - (y'*Cameraz)*Cameraz; % removing any components of z from y
    ynorm = norm(y);
end
% Normalise this non-zero y vector (made orthogonal to z)
Cameray = y/ynorm;
% Get a 3rd orthogonal unit vector from cross product of these two
Camerax = cross(Cameraz,Cameray);

% Create and populate T_cw matrix
T_cw = zeros(4);
T_cw(4,4) = 1; 
T_cw(1:3,1) = Camerax;
T_cw(1:3,2) = Cameray;
T_cw(1:3,3) = Cameraz;
T_cw(1:3,4) = CameraOrigin;
end
