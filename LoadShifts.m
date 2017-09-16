function varargout = LoadShifts(varargin)
% LOADSHIFTS MATLAB code for LoadShifts.fig
%      LOADSHIFTS, by itself, creates a new LOADSHIFTS or raises the existing
%      singleton*.
%
%      H = LOADSHIFTS returns the handle to a new LOADSHIFTS or the handle to
%      the existing singleton*.
%
%      LOADSHIFTS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LOADSHIFTS.M with the given input arguments.
%
%      LOADSHIFTS('Property','Value',...) creates a new LOADSHIFTS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LoadShifts_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LoadShifts_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LoadShifts

% Last Modified by GUIDE v2.5 14-Sep-2017 14:31:23

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LoadShifts_OpeningFcn, ...
                   'gui_OutputFcn',  @LoadShifts_OutputFcn, ...
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


% --- Executes just before LoadShifts is made visible.
function LoadShifts_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LoadShifts (see VARARGIN)

% set default values
handles.mass = 308;
handles.trackWidth = 1.244;
handles.wheelBase = 1.6;
handles.cogHeight = 0.25;
handles.frontalArea = 1;
handles.liftCoefficient = 3;
handles.speed = 39;
handles.weightDistribution = 0.47;
handles.accel = 1.28500744256168;
handles.cornering = 2.69712229269608;
handles.braking = 2.51808408686529;

% Choose default command line output for SteadyStateCornering
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LoadShifts wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LoadShifts_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.accel = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.cornering = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.braking = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.mass = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.cogHeight = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.trackWidth = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.wheelBase = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.frontalArea = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.liftCoefficient = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.speed = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
g = 9.8;

accelAndCornering = zeros(2, 2);
brakingAndCornering = zeros(2, 2);

downForce = getDownForce(handles.speed, handles.frontalArea, handles.liftCoefficient);

for i = 1:2
    for j = 1:2
        if (i == 1)
            accelAndCornering(i, j) = handles.mass * g / 2 * handles.weightDistribution + downForce / 4;
            brakingAndCornering(i, j) = handles.mass * g / 2 * handles.weightDistribution + downForce / 4;
        else
            accelAndCornering(i, j) = handles.mass * g / 2 * (1 - handles.weightDistribution) + downForce / 4;
            brakingAndCornering(i, j) = handles.mass * g / 2 * (1 - handles.weightDistribution) + downForce / 4;
        end
    end
end

% Suppose that the total force is in the direction theta
% then the longitudinal force is (maxLongitudinalForce * sin(theta) )
% and the lateral force is (maxLateralForce * cos(theta) )
% longitudial load shift is 
% (maxLongitudinalForce * sin(theta) ) * cogHeight / wheelBase
% lateral load shift is 
% (maxLateralForce * cos(theta) ) * cogHeight / trackWidth
% need to maximize the sum of these two. Just take a derivative. 
% Let A = maxLongitudinalForce / wheelBase
% Let B = maxLateralForce / trackWidth
% Max occurs at 
% sin(theta) = A / sqrt(A^2 + B^2)
% cos(theta) = B / sqrt(A^2 + B^2)
A = handles.accel / handles.wheelBase;
B = handles.braking / handles.wheelBase;
C = handles.cornering / handles.trackWidth;

sinTheta = A / sqrt(A^2 + C^2);
cosTheta = C / sqrt(A^2 + C^2);
sinPhi = B / sqrt(B^2 + C^2);
cosPhi = C / sqrt(B^2 + C^2);
% first deal with accel load shifts;
for i = 1:2
    for j = 1:2
        if (i == 1)
            accelAndCornering(i, j) = accelAndCornering(i, j) - (handles.accel * g * handles.mass * sinTheta ) * handles.cogHeight / handles.wheelBase;
        else
            accelAndCornering(i, j) = accelAndCornering(i, j) + (handles.accel * g * handles.mass * sinTheta ) * handles.cogHeight / handles.wheelBase;
        end
        if (j == 1)
            accelAndCornering(i, j) = accelAndCornering(i, j) - (handles.cornering * g * handles.mass * cosTheta ) * handles.cogHeight / handles.trackWidth;
        else
            accelAndCornering(i, j) = accelAndCornering(i, j) + (handles.cornering * g * handles.mass * cosTheta ) * handles.cogHeight / handles.trackWidth;
        end
    end
end

a = sqrt( (handles.accel * sinTheta)^2 + (handles.cornering * cosTheta)^2);

% now for the braking load shifts
for i = 1:2
    for j = 1:2
        if (i == 1)
            brakingAndCornering(i, j) = brakingAndCornering(i, j) + (handles.braking * g * handles.mass * sinPhi ) * handles.cogHeight / handles.wheelBase;
        else
            brakingAndCornering(i, j) = brakingAndCornering(i, j) - (handles.braking * g * handles.mass * sinPhi ) * handles.cogHeight / handles.wheelBase;
        end
        if (j == 1)
            brakingAndCornering(i, j) = brakingAndCornering(i, j) - (handles.cornering * g * handles.mass * cosPhi ) * handles.cogHeight / handles.trackWidth;
        else
            brakingAndCornering(i, j) = brakingAndCornering(i, j) + (handles.cornering * g * handles.mass * cosPhi ) * handles.cogHeight / handles.trackWidth;
        end
    end
end

b = sqrt( (handles.braking * sinPhi)^2 + (handles.cornering * cosPhi)^2);

disp("Acceleration and Cornering")
disp(accelAndCornering)
disp(a)
disp("Braking and Cornering")
disp(brakingAndCornering)
disp(b)



function f = getDownForce(velocity, frontArea, liftCoeff)
rho = 1.225; % air density
f = rho * liftCoeff * frontArea * velocity^2 / 2;

function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.weightDistribution = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
