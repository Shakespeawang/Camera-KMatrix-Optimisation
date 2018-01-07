function [ R ] = RodriguesRotation( RotationAxis, RotationAngle )
%RODRIGUESROTATION Uses the Rodrigues Rotation Formula to generate
%a rotation matrix, R, about an axis defined by RotationAxis and 
%by an angle of RotationAngle

%The RotationAngle is in radians

%Ensure that RotationAxis is of sufficient magnitude to normalise reliably 
%and is of the correct length (3) 
if length(RotationAxis) ~= 3
    error('Axis of rotation must have three elements')
end

if norm(RotationAxis) < eps
    error('Axis of rotation is of insufficient magnitude to normalise reliably')
end

%Normalise the axis of rotation
RotationAxis = RotationAxis/norm(RotationAxis);

%For clarity, extract the components of the axis of rotation
x = RotationAxis(1);
y = RotationAxis(2);
z = RotationAxis(3);

%Assemble the cross product matrix
K = [0, -z, y;
     z, 0, -x;   
     -y, x, 0];

%Define 3x3 identity matrix
I = eye(3);

%Compute the rotation matrix in accordance with the Rodrigues Rotation
%Formula
R = I + sin(RotationAngle)*K + (1 - cos(RotationAngle))*K^2;

end
