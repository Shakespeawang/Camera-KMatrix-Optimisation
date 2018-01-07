function [ KMatrix ] = KMatrix_Generate( Parameters )
%KMATRIX_GENERATE: This function generates the 'KMatrix' of the camera with
%the input being a single vector of 'Parameters' 

%Perform a check on the integrity of the input vector, it should have
% a length of 8  

if length(Parameters) == 8
    fprintf('Input vector (Parameters) length OK \n')
else 
    error('Input vector length incorrect')
end

%Extract values from Parameters and assign them to variables as follows: 

ChipWidth = Parameters(1);           %Width of the camera chip (x-pixels, integer)
ChipHeight = Parameters(2);          %Height of the camera chip (y-pixels, integer)
FocalLength = Parameters(3);         %Focal length of the lens (mm) 
PixelWidth = Parameters(4);          %Effective width of a pixel in the x direction (mm)
PixelHeight = Parameters(5);         %Effective height of a pixel in the y direction (mm)    
Skewness = Parameters(6);            %Skewness of the pixels (non-dimensional)
P_u = Parameters(7);                 %Principal direction offset in x (fraction of chip width)
P_v = Parameters(8);                 %Principal direction offset in y (fraction of chip height)


%Check that ChipWidth and Chip Height are integers
Check = Parameters(1)/fix(Parameters(1));
if Check ~= 1
    error('ChipWidth is not an integer')
end

Check2 = Parameters(2)/fix(Parameters(2));
if Check2 ~= 1 
    error('ChipHeight is not an integer')
end

%Check that all the parameters are within their respective ranges, using a function
%called 'TestRange'

    
TestRange(ChipWidth, 200, 4000, 'ChipWidth')
TestRange(ChipHeight, 300, 5000, 'ChipHeight')
TestRange(FocalLength, 1, 100, 'FocalLength')
TestRange(PixelWidth, 0.0001, 0.1, 'PixelWidth')
TestRange(PixelHeight, 0.0001, 0.1, 'PixelHeight')
TestRange(Skewness, -0.1, 0.1, 'Skewness')
TestRange(P_u, 0.25, 0.75, 'P_u')
TestRange(P_v, 0.25, 0.75, 'P_v')

fprintf('Input parameters within range \n')

%Calculate x focal length in pixels
F_x = FocalLength/PixelWidth;

%Calculate x focal length in pixels
F_y = FocalLength/PixelHeight;

%Construct KMatrix

KMatrix = [F_x, Skewness, P_u*ChipWidth;
            0,   F_y, P_v*ChipHeight;
            0,   0,    1];
        
end

