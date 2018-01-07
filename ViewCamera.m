function [ output_args ] = ViewCamera( ObjectPoints, CameraStructure, T_ow, T_cw )
%VIEWCAMERA Function shows the camera's view of an object described by
%ObjectPoints. 

%Function uses a structure CameraStructure which contains the KMatrix, 
%width and height of the camera, as well as 4x4 transformation matrices 
%T_ow and T_cw which detail the relationship between world, object and
%camera coordinate systems.

%Perform checks on the integrity of the inputs
s = size(ObjectPoints);
if isinteger(int8(s(2)/2)) == 0 || s(1) ~=4
    error('ObjectPoints has invalid dimensions')
end

s = size(CameraStructure.KMatrix);
if s(1) ~=3 || s(2) ~=3
    error('KMatrix has invalid dimensions')
end

%Check KMatrix is upper diagonal
if CameraStructure.KMatrix(2,1) ~=0 || CameraStructure.KMatrix(3,1) ~=0 ...
        || CameraStructure.KMatrix(3,2) ~=0
    error('KMatrix is not upper diagonal')
end

TestTransformMatrix(T_ow, 'T_ow')
TestTransformMatrix(T_cw, 'T_cw')

%END OF CHECKS ON INPUTS

%Transform the ObjectPoints from object coordinates firstly into world coordinates 
%and then into camera coordinates

%Note that the trasnformation from O:W -> C is the inverse of O:C ->W
%That is to say T_wc = (T_cw)^-1
ObjectPoints = T_ow*ObjectPoints;
ObjectPoints = T_cw\ObjectPoints; 

%Extract the 3-dimensional coordinates (now in the camera cooridinate
%system) from ObjectPoints
ObjectPoints_3D = ObjectPoints(1:3, :);

%Multiply by the K-Matrix to turn these 3-D points into 2-D homogeneous
%coordinates
KMatrix = CameraStructure.KMatrix;
ObjectPoints_2DH = KMatrix*ObjectPoints_3D;

%Scale each 2-D homogeneous point by its homogeneous multiplier
s = size(ObjectPoints_2DH);
p = s(2);
for i = 1:p;
    ObjectPoints_2DH(1:2, i) = ObjectPoints_2DH(1:2, i)/ObjectPoints_2DH(3,i);
end

%Extract the 2-D points to be connected by lines and then plotted
ObjectPoints_2D = ObjectPoints_2DH(1:2, :);

%Plot the lines, grouping pairs of points together and connecting them via
%a line
ChipWidth = CameraStructure.ChipWidth;
ChipHeight = CameraStructure.ChipHeight;

figure(1)
title('Simulated object as seen on camera chip')
clf
axis([0 ChipWidth 0 ChipHeight])
hold on 

for i=1:2:(p-1)
    plot([ObjectPoints_2D(1,i), ObjectPoints_2D(1,i+1)], ...
        [ObjectPoints_2D(2,i), ObjectPoints_2D(2, i+1)])
end

%Construct a 3-D visualisation of the camera and object to check the image
%generated above
CheckCameraView(ObjectPoints_3D, T_cw);

end

