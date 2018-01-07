function [ CameraStructure ] = BuildCamera( Parameters )
%BUILDCAMERA Function builds a structure 'CameraStructure' which contains
%the KMatrix, chip width and chip height of the camera. The data used to
%create the structure is initially in the Parameters vector

%The integrity of the 'Parameters' input vector is checked in
%KMatrix_Generate

CameraStructure.KMatrix = KMatrix_Generate(Parameters);

%For clarity, extract the chip width and chip height from 'Parameters'
ChipWidth = Parameters(1);
ChipHeight = Parameters(2);

CameraStructure.ChipWidth = ChipWidth; 
CameraStructure.ChipHeight = ChipHeight;


end

