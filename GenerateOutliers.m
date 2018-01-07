function [ Correspond ] = GenerateOutliers( Correspond, pOutlier, CameraStructure )
%GENERATEOUTLIERS Function generates outliers within matrix 'Correspond'
%with a probability 'pOutlier'. 'CameraStructure' contains the camera's
%chip width and chip height.

%An outlier is a point which cannot be accounted for by the camera's model
%and white noise

%Check validity of inputs
if size(Correspond, 1) ~= 4; 
    error('Correspond has incorrect dimensions')
end

if pOutlier < 0 || pOutlier > 1
    error('pOutlier must be between 0 and 1')
end

%END OF CHECKS ON INPUTS

for i = 1:length(Correspond)
    %Function 'rand' generates a random scaler between 0 and 1, so is
    %useful for a probability trial
    if rand < pOutlier
        %If the above condition is satisfied, the re-assign the [u,v] vector
        %in Correspond as an outlier
        %In this case, insert a random vector somewhere in the camera's
        %image
        Correspond(1,i) = rand*(CameraStructure.ChipWidth -1);
        Correspond(2,i) = rand*(CameraStructure.ChipHeight -1);
    end
end

end

