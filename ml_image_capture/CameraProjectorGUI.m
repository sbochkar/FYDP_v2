% Code for interfacing with camera taken from
% http://www.mathworks.com/support/solutions/en/data/1-1CBPW/

% This code requires PsychToolBox for controlling the projector
% http://psychtoolbox.org/PsychtoolboxDownload 
% and the MATLAB Image Acquisition Toolbox

function varargout = CameraProjectorGUI(varargin)
    % CAMERAPROJECTORGUI M-file for CameraProjectorGUI.fig
    %      CAMERAPROJECTORGUI, by itself, creates a new CAMERAPROJECTORGUI or raises the existing
    %      singleton*.
    %
    %      H = CAMERAPROJECTORGUI returns the handle to a new CAMERAPROJECTORGUI or the handle to
    %      the existing singleton*.
    %
    %      CAMERAPROJECTORGUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in CAMERAPROJECTORGUI.M with the given input arguments.
    %
    %      CAMERAPROJECTORGUI('Property','Value',...) creates a new CAMERAPROJECTORGUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before CameraProjectorGUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to CameraProjectorGUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help CameraProjectorGUI

    % Last Modified by GUIDE v2.5 26-Sep-2013 14:44:38

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @CameraProjectorGUI_OpeningFcn, ...
                       'gui_OutputFcn',  @CameraProjectorGUI_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end

    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end
    % End initialization code - DO NOT EDIT


% --- Executes just before CameraProjectorGUI is made visible.
function CameraProjectorGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CameraProjectorGUI (see VARARGIN)

% Choose default command line output for CameraProjectorGUI
    handles.output = hObject;

    % Turn off the scren options that run tests before each projection
    Screen('Preference','SkipSyncTests', 1);
    Screen('Preference','VisualDebugLevel', 0);
    % Create video object
    % Putting the object into manual trigger mode and then
    % starting the object will make GETSNAPSHOT return faster
    % since the connection to the camera will already have
    % been established. Some cameras take awhile to warm up.
    handles.video = videoinput('winvideo', 2, 'YUY2_1280x720'); %Use the imaqhwinfo function to determine the adaptors available on your system.
    %handles.video = videoinput('winvideo', 1);
    set(handles.video,'TimerPeriod', 0.05, ...
    'TimerFcn',['if(~isempty(gco)),'...
    'handles=guidata(gcf);'... % Update handles
    'image(getsnapshot(handles.video));'... % Get picture using GETSNAPSHOT and put it into axes using IMAGE
    'set(handles.cameraAxes,''ytick'',[],''xtick'',[]),'... % Remove tickmarks and labels that are inserted when using IMAGE
    'else '...
    'delete(imaqfind);'... % Clean up - delete any image acquisition objects
    'end']);
    triggerconfig(handles.video,'manual');
    handles.video.FramesPerTrigger = Inf; % Capture frames until we manually stop it
    handles.video.ReturnedColorspace = 'rgb';
    %Initialize capture Image button to be disabled (wait for user to start
    %cam)
    set(handles.captureImage,'Enable','off');
    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes CameraProjectorGUI wait for user response (see UIRESUME)
    uiwait(handles.CameraProjectorGUI);


% --- Outputs from this function are returned to the command line.
function varargout = CameraProjectorGUI_OutputFcn(hObject, eventdata, handles)
    % varargout cell array for returning output args (see VARARGOUT);
    % hObject handle to figure
    % eventdata reserved - to be defined in a future version of MATLAB
    % handles structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    handles.output = hObject;
    varargout{1} = handles.output;

% --- Executes on button press in startStopCamera.
function startStopCamera_Callback(hObject, eventdata, handles)
    % hObject handle to startStopCamera (see GCBO)
    % eventdata reserved - to be defined in a future version of MATLAB
    % handles structure with handles and user data (see GUIDATA)

    % Start/Stop Camera
    if strcmp(get(handles.startStopCamera,'String'),'Start Camera')
    % Camera is off. Change button string and start camera.
    set(handles.startStopCamera,'String','Stop Camera')
    start(handles.video)
    set(handles.captureImage,'Enable','on');
    else
    % Camera is on. Stop camera and change button string.
    set(handles.startStopCamera,'String','Start Camera')
    stop(handles.video)
    set(handles.captureImage,'Enable','off');
    end
    
    global count;
    count = 0;


% --- Executes on button press in captureImage.
function captureImage_Callback(hObject, eventdata, handles)
    % hObject handle to captureImage (see GCBO)
    % eventdata reserved - to be defined in a future version of MATLAB
    % handles structure with handles and user data (see GUIDATA)
     frame = getsnapshot(handles.video);
    
    % Use Screen function from PsychToolBox to project specified image
    % onto secondary monitor
    % From PsychToolBox Doc: 
    %screenindex 0 -> use all available screen
    %screenindex 1 -> use primary display
    %screenindex 2 -> use secondary display
    %...etc
    %There must be an easier way to display full screen images in MATLAB...
    screenIndex = 2;
    [w(1) sRect]=Screen('OpenWindow', screenIndex, 0,[],32,2);
    A = imread_rgb('checkerboard_1024_768.jpg'); %HARD CODED
    A = imresize(A, [768 1024]); %HARD CODED
    Screen('PutImage',w(1),A);
    Screen('Flip', w(1));
    pause(3.0); %HARD CODED
    
    global count;
    count = count + 1;
    frame = get(get(handles.cameraAxes,'children'),'cdata'); % The current displayed frame
    filename = strcat('calib_images/testFrame', num2str(count), '.bmp');
    imwrite(frame,filename); %HARD CODED
    %save('testframe.mat', 'frame');
    Screen('CloseAll');
    disp('Frame saved to file ''testFrame.bmp'''); %HARD CODED


% --- Executes when user attempts to close CameraProjectorGUI.
function CameraProjectorGUI_CloseRequestFcn(hObject, eventdata, handles)
    % hObject    handle to CameraProjectorGUI (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Remove any Image Acuisition objects
    % Note: not doing this and leaving the cam open when closing form will
    % cause you to lose reference of the cam object, and MATLAB must be
    % restarted.
    delete(hObject);
    delete(imaqfind);
