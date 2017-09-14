function varargout = StraightLineAcceleration(varargin)
% STRAIGHTLINEACCELERATION MATLAB code for StraightLineAcceleration.fig
%      STRAIGHTLINEACCELERATION, by itself, creates a new STRAIGHTLINEACCELERATION or raises the existing
%      singleton*.
%
%      H = STRAIGHTLINEACCELERATION returns the handle to a new STRAIGHTLINEACCELERATION or the handle to
%      the existing singleton*.
%
%      STRAIGHTLINEACCELERATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STRAIGHTLINEACCELERATION.M with the given input arguments.
%
%      STRAIGHTLINEACCELERATION('Property','Value',...) creates a new STRAIGHTLINEACCELERATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before StraightLineAcceleration_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to StraightLineAcceleration_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help StraightLineAcceleration

% Last Modified by GUIDE v2.5 04-Sep-2017 15:12:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @StraightLineAcceleration_OpeningFcn, ...
                   'gui_OutputFcn',  @StraightLineAcceleration_OutputFcn, ...
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


% --- Executes just before StraightLineAcceleration is made visible.
function StraightLineAcceleration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to StraightLineAcceleration (see VARARGIN)

% load data
handles.SRCoeffs = load("SR_Pacejka_Coeffs.mat");

% set default values
handles.tStep = 0.1;
handles.mass = 308;
handles.wheelBase = 1.6;
handles.cogHeight = 0.25;
handles.pressure = 70;
handles.frontalArea = 1.2;
handles.liftCoefficient = 1.5;
handles.dragCoefficient = 0.6;
handles.weightDistribution = 0.47;
handles.distance = 75;
handles.runUp = 0.305;
handles.power = 100;
handles.torque = 240;
handles.gearRatio = 4.1;

% Choose default command line output for StraightLineAcceleration
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes StraightLineAcceleration wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = StraightLineAcceleration_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

g = 9.8;
wheelRadius = 0.0254 * 9;
velocity = 0;
acceleration = 0;

position = -handles.runUp;
maxPosition = handles.distance;
time = 0;

effectiveRadius = wheelRadius / handles.gearRatio;

startTime = 0;

peakTireForce = 0;
peakOverturningMoment = 0;
peakAligningTorque = 0;
peakNormalForce = 0;

while position < maxPosition
    force = acceleration * handles.mass;
    loadTransfer = force * handles.cogHeight / handles.wheelBase;
    downForce = getDownForce(handles, velocity);
    
    tireLoad = handles.mass * g / 2 * (1 - handles.weightDistribution) + loadTransfer / 2 + downForce / 4;
    
    output = TireOutputs(handles, tireLoad);
    peakTireForce = max(peakTireForce, output(1) );
    peakOverturningMoment = max(peakOverturningMoment, output(2));
    peakAligningTorque = max(peakAligningTorque, output(3));
    peakNormalForce = max(peakNormalForce, tireLoad);
    
    tireForce = output(1);
    maxForce = tireForce * 2 * (2/3);
    
    forceAvailable = min( handles.torque / effectiveRadius , handles.power * 1000 / velocity );
    
    newForce = min(maxForce, forceAvailable);
    newForce = newForce - getDrag(handles, velocity);
    
    acceleration = newForce / handles.mass;
    velocity = velocity + acceleration * handles.tStep;
    disp(velocity);
    position = position + velocity * handles.tStep;
    
    time = time + handles.tStep;
    if (position < 0)
        startTime = time;
    end
end
totalTime = time - startTime;

format longG
disp("Time (seconds)");
disp(totalTime);
disp("Forces are for one rear tire. Forces on the front wheel are (probably) negligible.");
disp("Peak Normal Force (Newtons)");
disp(peakNormalForce);
disp("Peak tire longitudinal force (Newtons)");
disp(peakTireForce);
disp("Peak overturning moment (Newton-meters)");
disp(peakOverturningMoment);
disp("Peak aligning torque (Newton-Meters)");
disp(peakAligningTorque);
disp("Peak Acceleration (g)");
disp(peakNormalForce * 4 / 3 / handles.mass / g);


function f = getDownForce(handles, velocity)
rho = 1.225; % air density
f = rho * handles.liftCoefficient * handles.frontalArea * (velocity^2);

function f = getDrag(handles, velocity)
rho = 1.225;
f = rho * handles.dragCoefficient * handles.frontalArea * (velocity^2);

function output = TireOutputs(handles, tireLoad)
pressure = handles.pressure * 0.145038;
camber = 0;
normalForce = tireLoad / 4.4475 / 50;
slipAngle = 0;

pressureList = [8, 10, 12, 14];
camberList = [0, 2, 4];
normalForceList = [1, 3, 4, 5];
slipAngleList = [0, 3, 6];

i = 1;
while (i < length(pressureList)-1 && pressureList(i+1) < pressure)
    i = i + 1;
end
j = 1; 
while (j < length(camberList)-1 && camberList(j+1) < camber)
    j = j + 1;
end
k = 1;
while (k < length(normalForceList)-1 && normalForceList(k+1) < normalForce)
    k = k + 1;
end
m = 1;
while (m < length(slipAngleList)-1 && slipAngleList(m+1) < slipAngle)
    m = m + 1;
end

%disp( [ normalForceList(k), normalForce, normalForceList(k+1) ] );
    
x = zeros(1, 4);
overturningMoment = 0;
aligningTorque = 0;

%totalWeight = 0;

for a = 0:1
    if (a == 0)
        pressureWeight = (pressureList(i+1) - pressure) / (pressureList(i+1) - pressureList(i));
    else
        pressureWeight = (pressure - pressureList(i)) / (pressureList(i+1) - pressureList(i));
    end
    for b = 0:1
        if (b == 0)
            camberWeight = (camberList(j+1) - camber) / (camberList(j+1) - camberList(j));
        else
            camberWeight = (camber - camberList(j)) / (camberList(j+1) - camberList(j));
        end
        for d = 0:1
            if (d == 0)
                normalForceWeight = (normalForceList(k+1) - normalForce) / (normalForceList(k+1) - normalForceList(k));
            else
                normalForceWeight = (normalForce - normalForceList(k)) / (normalForceList(k+1) - normalForceList(k));
            end
            
            for e = 0:1
                if (e == 0)
                    slipAngleWeight = (slipAngleList(m+1) - slipAngle) / (slipAngleList(m+1) - slipAngleList(m));
                else
                    slipAngleWeight = (slipAngle - slipAngleList(m)) / (slipAngleList(m+1) - slipAngleList(m));
                end
            
            
                pointWeight = pressureWeight * camberWeight * normalForceWeight * slipAngleWeight;
                %disp(pointWeight);
                
                %totalWeight = totalWeight + pointWeight;
                
                p = pressureList(i + a) / 2 - 4;
                c = camberList(j + b) / 2;
                f = normalForceList(k + d) - 2;
                s = slipAngleList(m + e) / 3;
                if (f < 0)
                    f = 0;
                end
                index = p*3*4*3 + c*3*4 + f*3 + s + 1;
                coeffData = handles.SRCoeffs.coEff(index);
                %disp(coeffData.force);
                %disp(coeffData.coeff);
                x = x + coeffData.coeff * pointWeight;
                overturningMoment = overturningMoment + coeffData.overturning * pointWeight;
                aligningTorque = aligningTorque + coeffData.aligning * pointWeight;
            end
        end
    end
end

% disp(totalWeight);

%{
SABound = 0.3; %radians
SAStep = 0.0001;

%disp("x");
%disp(x);
y = @(xdata) x(3)*sin(x(2)*atan(x(1)*xdata - x(4)*(x(1)*xdata - atan(x(1)*xdata))));
xdata = 0:SAStep:SABound;
% factor of cos(xdata) is unlikely to make a difference, as stated in RCVD
% However, might as well have it in anyways. 
sideForces = y(xdata) .* cos(xdata); 
%disp(sideForces);
f = max(sideForces);
%}

% under current model, just taking the third coefficient is sufficient. 
f = abs(x(3)); % might be a sign problem, just to be sure. 
% disp(f);
output = [f, overturningMoment, aligningTorque];


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.tStep = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.mass = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.wheelBase = str2double(get(hObject, 'String'));
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

handles.cogHeight = str2double(get(hObject, 'String'));
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

handles.pressure = str2double(get(hObject, 'String'));
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

handles.frontalArea = str2double(get(hObject, 'String'));
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

handles.liftCoefficient = str2double(get(hObject, 'String'));
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

handles.dragCoefficient = str2double(get(hObject, 'String'));
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

handles.weightDistribution = str2double(get(hObject, 'String'));
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

handles.distance = str2double(get(hObject, 'String'));
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

handles.runUp = str2double(get(hObject, 'String'));
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



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.power = str2double(get(hObject, 'String'));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
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

handles.torque = str2double(get(hObject, 'String'));
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



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.gearRatio = str2double(get(hObject, 'String'));
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
