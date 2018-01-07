function [  ] = CheckCameraView2( ObjectPoints_3D, T_cw, CameraDistance )
%TESTCAMERAVIEW Function plots, on a 3D grid, the position of the object being 
%viewed (defined in ObjectPoints_3D) as well as the position and orientation 
%of the camera (defined by T_cw)

%Find the number of rows in ObjectPoints_3D
s = size(ObjectPoints_3D, 1);

%Check the integrity of the inputs
if s ~=3
    error('ObjectPoints_3D has invalid dimensions, CheckCameraView2')
end

TestTransformMatrix(T_cw, 'T_cw')

%END OF CHECKS ON INPUTS

%Extract the orientation (of the z-axis) and position of the camera
position = T_cw(1:3, 4);
orientation = T_cw(1:3, 3);

%Draw the z-axis of the camera coordinate system (the orientation vector
%is scaled by CameraDistance/0.9)
v1 = position;
v2 = (CameraDistance/0.9)*orientation;

%Use the 'quiver' function in Matlab to show the z-axis as an arrow
figure(1)
clf
quiver3(v1(1), v1(2), v1(3), v2(1), v2(2), v2(3))
grid on 
axis equal

hold on 

%Also show the object to be viewed in 3-D
fill3(ObjectPoints_3D(1,:), ObjectPoints_3D(2,:), ObjectPoints_3D(3,:), 'g')
title('Simulation of camera looking at the grid')

end

