function A = imread_rgb(name)
try
    [A map alpha] = imread(name);
catch
    % Format not recognized by imread, so create a red cross (along diagonals)
    A = eye(101) | diag(ones(100, 1), 1) | diag(ones(100, 1), -1);
    A = (uint8(1) - uint8(A | flipud(A))) * uint8(255);
    A = cat(3, zeros(size(A), 'uint8')+uint8(255), A, A);
    return
end
A = A(:,:,:,1); % Keep only first frame of multi-frame files
if ~isempty(map)
    map = uint8(map * 256 - 0.5); % Convert to uint8 for storage
    A = reshape(map(uint32(A)+1,:), [size(A) size(map, 2)]); % Assume indexed from 0
elseif size(A, 3) == 4
    if lower(name(end)) == 'f'
        % TIFF in CMYK colourspace - convert to RGB
        if isfloat(A)
            A = A * 255;
        else
            A = single(A);
        end
        A = 255 - A;
        A(:,:,4) = A(:,:,4) / 255;
        A = uint8(A(:,:,1:3) .* A(:,:,[4 4 4]));
    else
        % Assume 4th channel is an alpha matte
        alpha = A(:,:,4);
        A = A(:,:,1:3);
    end
end
if ~isempty(alpha)
    % Apply transprency over a grey checkerboard pattern
    if isa(alpha, 'uint8')
        alpha = double(alpha) / 255;
    end
    A = double(A) .* alpha(:,:,ones(1, size(A, 3)));
    sqSz = max(size(alpha));
    sqSz = floor(max(log(sqSz / 100), 0) * 10 + 1 + min(sqSz, 100) / 20);
    grid = repmat(85, ceil(size(alpha) / sqSz));
    grid(2:2:end,1:2:end) = 171;
    grid(1:2:end,2:2:end) = 171;
    grid = kron(grid, ones(sqSz));
    alpha = grid(1:size(A, 1),1:size(A, 2)) .* (1 - alpha);
    A = uint8(A + alpha(:,:,ones(1, size(A, 3))));
end
return