%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialiations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
%Screen command is from Psychtoolbox. Turn off debug functionality
Screen('Preference','SkipSyncTests', 1);
Screen('Preference','VisualDebugLevel', 0);
Screen('Preference','SuppressAllWarnings',1);
%Add dependent directories
addpath('./utilities','./drivers','./calib');
%Get calibration parameters fromm get_parameters.m script inside calib dir.
get_parameters;

% Reset Matlab environment.
% Note: Keep handles to previous projector screen, camera(s), and codes.
if exist('window','var')
   keep('window','rect','height','width','camera');
   for i = 1:length(camera)
      stop(camera{i}); start(camera{i});
   end
end

% Set structured lighting parameters.
objName      = 'test'; % object name (should correspond to a data directory)
seqType      = 'Gray'; % structured light sequence type ('Gray' or 'bin')
dSampleProj  = 1;      % downsampling factor (i.e., min. system resolution)
projValue    = 128;    % Gray code intensity
minContrast  = 0.2;    % minimum contrast threshold (for Gray code pattern)
screenIndex  = 2;      % index of projector display (1 = first, 2 = second, etc.)
frameDelay   = 0.1;    % frame delay (in seconds)

% Set reconstruction parameters.
dSamplePlot = 100;     % down-sampling rate for Matlab point cloud display
distReject  = inf;     % rejection distance (for outlier removal)
saveResults = true;    % enable/disable results output

% Define camera(s) for use with the Image Acquistion Toolbox.
% Note: Type 'imaqhwinfo' at comment prompt for camera properties.
%       For example, 'info = imaqhwinfo('winvideo');'.
camName     = {'winvideo'};
camID       = [1];
camFormat   = {'YUY2_1280x720'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate Gray Code Patterns
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get projector display properties.
% Note: Assumes the Matlab Psychtoolbox is installed.
if ~exist('window','var')
   window = Screen('OpenWindow',screenIndex,projValue*[1 1 1]);
   rect   = Screen('Rect',window);
   height = rect(4); width  = rect(3);
end
   
% Generate vertical and horizontal Gray code stripe patterns.
% Note: P{j} contains the Gray code patterns for "orientation" j.
%       offset(j) is the integer column/row offset for P{j}.
%       I{j,i} are the OpenGL textures corresponding to bit i of P{j}.
%       J{j,i} are the OpenGL textures of the inverse of I{j,i}.
if ~exist('I','var') || ~exist('J','var')
   if strcmp(seqType,'Gray')
      [P,offset] = graycode(width/dSampleProj,height/dSampleProj);
   else
      [P,offset] = bincode(width/dSampleProj,height/dSampleProj);
   end
   I = {}; J = {};
   for j = 1:2
      for i = 1:size(P{j},3)
         I{j,i} = Screen('MakeTexture',window,projValue*imresize(P{j}(:,:,i),dSampleProj));
         J{j,i} = Screen('MakeTexture',window,projValue*imresize(1-P{j}(:,:,i),dSampleProj));
      end
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Project Gray Code Patterns
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define serial object for turntablen
tableSerial = serial('COM3', 'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none', 'StopBits', 1, 'FlowControl', 'none');
%right now, speed of 500R, takes 3200 steps and 10s to go around 360
numberAngles = 72;
totalSteps = 3200;
totalTime = 10;
step = round(totalSteps/numberAngles);
pauseTime = totalTime/numberAngles;
for scanIndex = 0:numberAngles-1
    %Open Serial Port Controlling Turntable
    fopen(tableSerial);
    %fprintf(tableSerial,'100R'); %speed
    command = strcat(num2str(step*scanIndex),'G');
    fprintf(tableSerial,command);
    pause(pauseTime);
    %tableSerial.BytesAvailable
    %ID = fgets(tableSerial)
    fclose(tableSerial);

    
    % Initialize camera(s) and allocate storage.
    % Note: Make sure to optimize the camera settings.
    %       A{j,i} is the camera image of I{j,i}.
    %       B{j,i} is the camera image of J{j,i}.

    if ~exist('camera','var')
       camera = camInit(camName,camID,camFormat);
    end
    A = cell(size(I));
    B = cell(size(I));

    if ~exist('allOn','var') || ~exist('allOff','var')
       allOn  = Screen('MakeTexture',window,projValue*ones(height,width,'uint8'));
       allOff = Screen('MakeTexture',window,zeros(height,width,'uint8'));
    end
    Screen('CopyWindow',allOn,window,rect,rect);
    Screen('Flip',window);

   % pause(5);
    
    
    % Initialize projector display and acquire "all on" and "all off" images.
    %Screen('HideCursorHelper',window);
    Screen('CopyWindow',allOn,window,rect,rect);
    Screen('Flip',window); 
    pause(frameDelay);
    T{1} = camCapture(camera);
    Screen('CopyWindow',allOff,window,rect,rect);
    Screen('Flip',window); 
    pause(frameDelay);
    T{2} = camCapture(camera);

    
    % Display/capture image(s) using projector and PGR camera(s).
    disp('Scanning object...');
    disp('+ Displaying structured light sequence...');
    for j = 1:size(I,1)
        for i = 1:size(I,2)
            x = i;
            y = j;
            % Display and capture current Gray code stripe pattern.
            Screen('CopyWindow',I{j,i},window,rect,rect);
            Screen('Flip',window); 
            pause(frameDelay);
            A{j,i} = camCapture(camera);
            Screen('CopyWindow',J{j,i},window,rect,rect);
            Screen('Flip',window); 
            pause(frameDelay);
            B{j,i} = camCapture(camera);

        end
    end

    % Disable the fullscreen display.
    %Screen('ShowCursorHelper',window);
    Screen('CopyWindow',allOff,window,rect,rect);
    Screen('Flip',window);
    %delete camera
    % clear I J;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Decode Gray Codes
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Estimate column/row label for each pixel (i.e., decode Gray codes).
    % Note: G{j,k} is the estimated Gray code for "orientation" j and camera k.
    %       D{j,k} is the integer column/row estimate.
    %       M{j,k} is the per-pixel mask (i.e., pixels with enough contrast).
    disp('+ Recovering projector rows/columns from structured light sequence...');
    G = cell(size(A,1),length(camera));
    D = cell(size(A,1),length(camera));
    M = cell(size(A,1),length(camera));
    C = inv([1.0 0.956 0.621; 1.0 -0.272 -0.647; 1.0 -1.106 1.703]);
    C = C(1,:)';

    for k = 1:length(camera)
       for j = 1:size(A,1)
          G{j,k} = zeros(size(T{1}{1},1),size(T{1}{1},2),size(A,2),'uint8');
          M{j,k} = false(size(T{1}{1},1),size(T{1}{1},2));
          for i = 1:size(A,2)

             % Convert image pair to grayscale.
             %grayA = rgb2gray(im2double(A{j,i}{k}));
             %grayB = rgb2gray(im2double(B{j,i}{k}));
             grayA = imlincomb(C(1),A{j,i}{k}(:,:,1),...
                               C(2),A{j,i}{k}(:,:,2),...
                               C(3),A{j,i}{k}(:,:,3),'double');
             grayB = imlincomb(C(1),B{j,i}{k}(:,:,1),...
                               C(2),B{j,i}{k}(:,:,2),...
                               C(3),B{j,i}{k}(:,:,3),'double');

             % Eliminate all pixels that do not exceed contrast threshold.
             M{j,k}(abs(grayA-grayB) >= 255*minContrast) = true;

             % Estimate current bit of Gray code from image pair.        
             bitPlane = zeros(size(T{1}{1},1),size(T{1}{1},2),'uint8');
             bitPlane(grayA(:,:) >= grayB(:,:)) = 1;
             G{j,k}(:,:,i) = bitPlane;

          end
          if strcmp(seqType,'Gray')
             D{j,k} = gray2dec(G{j,k})-offset(j);
          else
             D{j,k} = bin2dec(G{j,k})-offset(j);
          end
          D{j,k}(~M{j,k}) = NaN;
       end
    end

    % Eliminate invalid column/row estimates.
    % Note: This will exclude pixels if either the column or row is missing.
    %       D{j,k} is the column/row for "orientation" j and camera k.
    %       mask{k} is the overal per-pixel mask for camera k.
    mask = cell(length(camera));
    for k = 1:length(camera)
       mask{k} = M{1,k};
       for j = 1:size(D,1)
          if j == 1
             D{j,k}(D{j,k} > width) = NaN;
          else
             D{j,k}(D{j,k} > height) = NaN;
          end
          D{j,k}(D{j,k} < 1) = NaN;
          for i = 1:size(D,1)
             D{j,k}(~M{i,k}) = NaN;
             mask{k} =  mask{k} & M{i,k};
          end
       end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Depth Extraction Using Line-Plane Intersection
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Reconstruct 3D points using intersection with illumination plane(s).
    % Note: Reconstructs from all cameras in the first camera coordinate system.
    vertices = cell(1,length(Nc));
    colors   = cell(1,length(Nc));
    disp('+ Reconstructing 3D points...');
    %for i = 1:length(Nc)
        i = 1;
       idx       = find(~isnan(D{1,i}) & ~isnan(D{2,i}));
       [row,col] = ind2sub(size(D{1,i}),idx);
       npts      = length(idx);
       colors{i} = 0.65*ones(npts,3);
       Rc        = im2double(T{1}{i}(:,:,1));
       Gc        = im2double(T{1}{i}(:,:,2));
       Bc        = im2double(T{1}{i}(:,:,3));
       vV = intersectLineWithPlane(repmat(Oc,1,npts),Nc(:,idx),wPlaneCol(D{1,i}(idx),:)');
       vH = intersectLineWithPlane(repmat(Oc,1,npts),Nc(:,idx),wPlaneRow(D{2,i}(idx),:)');
       vertices{i} = vV';
       rejectIdx = find(sqrt(sum((vV-vH).^2)) > distReject);
       vertices{i}(rejectIdx,1) = NaN;
       vertices{i}(rejectIdx,2) = NaN;
       vertices{i}(rejectIdx,3) = NaN;
       colors{i}(:,1) = Rc(idx);
       colors{i}(:,2) = Gc(idx);
       colors{i}(:,3) = Bc(idx);
    %end
    filename = strcat('scan',num2str(scanIndex),'.ply');
    write_ply( -1.*vertices{1}, 'empty_string',filename, 'binary_big_endian');
end
stop(camera{1});
clear tableSerial;
delete imaqfind;
delete camera;
Screen('CloseAll');
system(['ScanIt_System.exe -f scan -n ', num2str(numberAngles)])

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Part III: Display reconstruction results and save results.
% 
% % Display status.
% disp('+ Displaying results and exporting VRML model...');
% 
% % Display project/camera calibration results.
% procamCalibDisplay;
% 
% % Display the recovered 3D point cloud (with per-vertex color).
% % Note: Convert to indexed color map for use with FSCATTER3.
% for i = 1:1%length(Nc)
%    C = reshape(colors{i},[size(colors{i},1) 1 size(colors{i},2)]);
%    [C,cmap] = rgb2ind(C,256);
%    hold on;
%       fscatter3(vertices{i}(1:dSamplePlot:end,1),...
%                 vertices{i}(1:dSamplePlot:end,3),...
%                -vertices{i}(1:dSamplePlot:end,2),...
%                 double(C(1:dSamplePlot:end)),cmap);
%    hold off;
%    axis tight; drawnow;
% end
% 
% % Create output data directories (if they do not exist).
% if ~exist(['./data/',seqType],'dir')
%    mkdir(['./data/',seqType]);
% end
% if ~exist(['./data/',seqType,'/',objName],'dir')
%    mkdir(['./data/',seqType,'/',objName]);
% end
% for i = 1:length(camera)
%    if ~exist(['./data/',seqType,'/',objName,'/v',int2str(i)],'dir');
%       mkdir(['./data/',seqType,'/',objName,'/v',int2str(i)]);
%    end
% end
% 
% % Export colored point cloud as a VRML file.
% % Note: Interchange x and y coordinates for j3DPGP.
% clear idx; mergedVertices = []; mergedColors = [];
% idx{i} = find(~isnan(vertices{i}(:,1)));
% vertices{i}(:,2) = -vertices{i}(:,2);
% 
% % Testing out ply_write function
 %write_ply( -1.*vertices{1}, 'empty_string','one_view_pc.ply');
 %Screen('CloseAll');

% % %Project vertices to depth map
%  rotatedVertices = zeros(399192,3);
% % x = 1; y = 2; z = 3;
% % %Rc_1_cam = [ 0.049112 	 0.919185 	 0.390752; ...
% % %             0.990403 	 0.005785 	 -0.138088; ...
% % %            -0.129189 	 0.393784 	 -0.910079 ];
% % %Tc_1_cam = [ -78.893576 	 -284.966378 	 967.047430 ]';
% % 
% for vertIndex = 1:399192
%     vX = -vertices{1}(vertIndex,x);
%     vY = -vertices{1}(vertIndex,y);
%     vZ = -vertices{1}(vertIndex,z);
%     rotatedVertices(vertIndex,x) = 0.049112 * vX + 0.919185 * vY + 0.390752  * vZ - 78.893576;
%     rotatedVertices(vertIndex,y) = 0.990403 * vX + 0.005785 * vY + -0.138088 * vZ - 284.966378;
%     rotatedVertices(vertIndex,z) = -0.129189* vX + 0.393784 * vY + -0.910079 * vZ + 967.047430;
% end
% depthMap = zeros(720,1280);
% 
% for index = 1:length(rotatedVertices)
%     vX = rotatedVertices(index,x);
%     vY = rotatedVertices(index,y);
%     vZ = rotatedVertices(index,z);
%     pixelX = (vX*fc_cam(1)/vZ + cc_cam(1));
%     pixelY = (vY*fc_cam(2)/vZ + cc_cam(2));
%     pixelVal = vZ;
%     if(pixelX > 1 & pixelX < 1281 & pixelY > 1 & pixelY < 721 & abs(pixelVal) < 3000)
%         depthMap(uint32(pixelX),uint32(pixelY)) = pixelVal;
%     end
% end



% vrmlPoints(['./data/',seqType,'/',objName,'/v',int2str(i),'.wrl'],...
%   vertices{i}(idx{i},[1 2 3]),colors{i}(idx{i},:));
% mergedVertices = [mergedVertices; vertices{i}(idx{i},[1 2 3])];
% mergedColors = [mergedColors; colors{i}(idx{i},:)];
% 
% % Save captured structured lighting sequences.
% if saveResults
%    disp('+ Saving structured light image sequence...');
%    for camIdx = 1:1%length(camera)
%       dataDir = ['./data/',seqType,'/',objName,'/v',int2str(camIdx),'/'];
%       imwrite(T{1}{camIdx},[dataDir,num2str(1,'%0.02d'),'.bmp']);
%       imwrite(T{2}{camIdx},[dataDir,num2str(2,'%0.02d'),'.bmp']);
%       frameIdx = 3;
%       for j = 1:2
%          for i = 1:size(A,2)
%             imwrite(A{j,i}{camIdx},[dataDir,num2str(frameIdx,'%0.02d'),'.bmp']);
%             frameIdx = frameIdx + 1;
%             imwrite(B{j,i}{camIdx},[dataDir,num2str(frameIdx,'%0.02d'),'.bmp']);
%             frameIdx = frameIdx + 1;
%          end
%       end
%    end
% end
% disp(' ');
