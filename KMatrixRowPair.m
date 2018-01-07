function [ Rows ] = KMatrixRowPair( H )
%KMATRIXROWPAIR Function takes data from a homography, H, and uses it to
%build two rows in a Regressor matrix.

%Check validity of input homography
if size(H) ~= [3,3];
    error('Homography input to KMatrixRowPair has invalid dimensions')
end

%Assign space for the rows
Rows = zeros(2,6);

%Extract the required elements from H for clarity 
h11 = H(1,1);
h12 = H(1,2);
h21 = H(2,1);
h22 = H(2,2);
h31 = H(3,1);
h32 = H(3,2);

Rows(1,:) = [h11^2 - h12^2, 2*(h11*h21 - h12*h22), 2*(h11*h31 - h12*h32),...
            h21^2 - h22^2, 2*(h21*h31 - h22*h32), h31^2 - h32^2];

Rows(2,:) = [h11*h12, h11*h22 + h21*h12, h11*h32 + h31*h12...
             h21*h22, h21*h32 + h31*h22, h31*h32];

end

