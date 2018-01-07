function [ T_ow ] = PositionObject( )
%POSITIONOBJECT Function places an object coordinate system randomly with 
%respect to world coordinates with a random orientation. 

%This location and orientation is then used to define a 4x4 homogeneous 
%transformation, T_ow, which transforms a homogeneous Point from
%object coordinates to world coordinates.

%The steps are seperated as above for clarity

%NOTE: Subscript T_ow means transformation from 'o' (object) to 'w' (world)
%coordinates

%Assign space for the 4x4 transformation matrix
T_ow = zeros(4,4);

%Define the origin of the object coordinate system, relative to the world
%origin. Scale of 1000 corresponds a length of 1m

position = 1000*rand(3,1);
orientation = RandomRotationMatrix;

%Assemble T_ow
T_ow(1:3, 1:3) = orientation;
T_ow(1:3, 4) = position;

%Bottom row of T_ow is [0,0,0,1] to turn the matrix into a homogeneous
%transformation
T_ow(4,4) = 1;

end

