function [ PercentageDifference ] = FindPercentageError( True, Estimate )
%FINDPERCENTAGEERROR Function find the element-wise percentage difference
%between the two nxn matrices (with respect to the 'True' matrix)

%'n' is the length of True
n = length(True);

%Assign space for the difference matrix 
PercentageDifference = zeros(n);

%Assign
for i=1:n
    for j=1:n
        PercentageDifference(i,j) = (True(i,j)-Estimate(i,j))/True(i,j);
        PercentageDifference(i,j) = 100*PercentageDifference(i,j);
    end
end

end

