function dispM = get_disparity(im1, im2, maxDisp, windowSize)
% GET_DISPARITY creates a disparity map from a pair of rectified images im1 and
%   im2, given the maximum disparity MAXDISP and the window size WINDOWSIZE.
mask = ones(windowSize, windowSize);

dispM = zeros(size(im1));
minDispM = ones(size(im1)) * 1000000;


% conv2 in combination with imtranslate which shifts the contents of the
% array left or right (displacement)

for d = 0:maxDisp
    currentDispM = conv2((im1 - imtranslate(im2, [d, 0], 'FillValues', 255)) .^ 2, mask, 'same');
    dispM(currentDispM < minDispM) = d;
    minDispM = min(minDispM, currentDispM);
end