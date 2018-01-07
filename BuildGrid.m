function [ GridPoints ] = BuildGrid( GridWidth, GridIncrement )
%BUILDGRID Function returns homogeneous Points (in 3-D) defining all
%corners in a square grid with with square tiles. The grid has side length
%GridWidth and each tile has side length GridIncrement.

%Check the validity of the inputs

%Calculate how many tiles in one row of the grid. Make sure the result is
%an integer.
N = GridWidth/GridIncrement;

if isinteger(int8(N))== 0 
    error('GridWidth/GridIncrement is not an integer')
end 

%(N+1)^2 is how many Points are required to define the corners of each tile 
%in the grid. Define this number
N2 = (N+1)^2;

%Assign space for a 4xN2 matrix which will contain the Points of the grid
GridPoints = zeros(4,N2);

%Set the homogeneous multiplier of each Point on the grid to 1
GridPoints(4,:) = ones(1,N2);

%Define a grid in the x-y plane centred on the origin by defining
%the x and y components of the grid

%The grid is defined by all permutations of the following two vectors
x = linspace(-GridWidth/2, GridWidth/2, N+1);
y = linspace(-GridWidth/2, GridWidth/2, N+1);

for j=1:N+1;
    %Here we systematically fill in sections of length N+1 of GridPoints by 
    %first defining all points with x cooridinates x(1), then x(2), and so on
    GridPoints(1,(j-1)*(N+1)+1 : j*(N+1)) = x(j);
    GridPoints(2,(j-1)*(N+1)+1 : j*(N+1)) = y;
end

end

