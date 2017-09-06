function varargout = SteadyStateCornering(varargin)
% STEADYSTATECORNERING MATLAB code for SteadyStateCornering.fig
%      STEADYSTATECORNERING, by itself, creates a new STEADYSTATECORNERING or raises the existing
%      singleton*.
%
%      H = STEADYSTATECORNERING returns the handle to a new STEADYSTATECORNERING or the handle to
%      the existing singleton*.
%
%      STEADYSTATECORNERING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STEADYSTATECORNERING.M with the given input arguments.
%
%      STEADYSTATECORNERING('Property','Value',...) creates a new STEADYSTATECORNERING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SteadyStateCornering_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SteadyStateCornering_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SteadyStateCornering

% Last Modified by GUIDE v2.5 18-Aug-2017 08:18:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SteadyStateCornering_OpeningFcn, ...
                   'gui_OutputFcn',  @SteadyStateCornering_OutputFcn, ...
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


% --- Executes just before SteadyStateCornering is made visible.
function SteadyStateCornering_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SteadyStateCornering (see VARARGIN)

% load data
handles.SACoeffs = load("SA_Pacejka_Coeffs.mat");

% set default values
handles.vStep = 0.1;
handles.mass = 296;
handles.trackWidth = 1.323;
handles.cogHeight = 0.25;
handles.pressure = 70;
handles.frontalArea = 1.22;
handles.liftCoefficient = 4.4;
handles.radius = 15;
handles.weightDistribution = 0.47;

% Choose default command line output for SteadyStateCornering
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SteadyStateCornering wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SteadyStateCornering_OutputFcn(hObject, eventdata, handles) 
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
velocity = 0;

neededSideForce = 0;
maxSideForce = 1;
while (neededSideForce < maxSideForce)
    velocity = velocity + handles.vStep;
    acceleration = (velocity / 3.6)^2 / handles.radius;
    neededSideForce = acceleration * handles.mass;
    loadTransfer = neededSideForce * handles.cogHeight / handles.trackWidth;
    downForce = getDownForce(velocity, handles.frontalArea, handles.liftCoefficient);
    insideTiresLoad = handles.mass * g / 2 - loadTransfer + downForce / 2;
    outsideTiresLoad = handles.mass * g / 2 + loadTransfer + downForce / 2;
    
    tireLoads = zeros(2, 2);
    sideForce = zeros(2, 2);
    overturningMoment = zeros(2, 2);
    aligningTorque = zeros(2, 2);
    
    for i = 1:2
        if i == 1
            weightRatio = handles.weightDistribution;
        else
            weightRatio = 1 - handles.weightDistribution;
        end
        
        for j = 1:2
            if j == 1
                load = insideTiresLoad;
            else
                load = outsideTiresLoad;
            end
            
            tireLoads(i, j) = load * weightRatio;
            output = TireOutputs(handles, tireLoads(i, j));
            %disp(output);
            
            sideForce(i, j) = output(1);
            overturningMoment(i, j) = output(2);
            aligningTorque(i, j) = output(3);
        end
    end
    
    frontTiresSideForce = sum(sideForce(1, :));
    rearTiresSideForce = sum(sideForce(2, :));
    
    moment = min( frontTiresSideForce * (1 - handles.weightDistribution), rearTiresSideForce * handles.weightDistribution );
    
    maxSideForce = moment / (1 - handles.weightDistribution) + moment / handles.weightDistribution; 

    % factor of two thirds from the contents guide in round 5 tire data
    % page 8, test comments
    maxSideForce = maxSideForce * (2/3); 
    % also, removing this line seems to make the car pull 6G in a turn. 
end
format longG

disp("Tire Loads (Newtons). Left column is inside of the turn. Top row repersents front tires")
disp( tireLoads )
disp("Tire Max SideForces (Newtons). Left column is inside of the turn. Top row repersents front tires")
disp("Values not balanced for steady state cornering")
disp( sideForce )
disp("Overturning Moment (Newton-meters)")
disp( overturningMoment )
disp("Aligning Torque (Newtown-meters)")
disp( aligningTorque )
disp("Maximum Velocity Reached (km per hour) ")
disp(velocity)
disp("Total side force acting on car (Newtons)")
disp( maxSideForce )
disp("Acceleration (g)")
disp( maxSideForce / handles.mass / 9.8 )



function f = getDownForce(velocity, frontArea, liftCoeff)
rho = 1.225; % air density
f = rho * liftCoeff * frontArea * (velocity / 3.6)^2;


function output = TireOutputs(handles, tireLoad)
pressure = handles.pressure * 0.145038;
camber = 0;
normalForce = tireLoad / 4.4475 / 50;

pressureList = [8, 10, 12, 14];
camberList = [0, 1, 2, 3, 4];
normalForceList = [1, 2, 3, 4, 5];

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

%disp( [ normalForceList(k), normalForce, normalForceList(k+1) ] );
    
x = zeros(1, 4);
overturningMoment = 0;
aligningTorque = 0;

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
            
            pointWeight = pressureWeight * camberWeight * normalForceWeight;
            %disp(pointWeight);
            p = pressureList(i + a) / 2 - 4;
            c = camberList(j + b);
            f = normalForceList(k + d) - 1;
            index = p*25 + c*5 + f + 1;
            coeffData = handles.SACoeffs.coEff(index);
            %disp(coeffData.coeff);
            x = x + coeffData.coeff * pointWeight;
            overturningMoment = overturningMoment + coeffData.overturning * pointWeight;
            aligningTorque = aligningTorque + coeffData.aligning * pointWeight;
        end
    end
end

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
output = [f, overturningMoment, aligningTorque];
%disp(output);


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.vStep = str2double(get(hObject, 'String'));
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

handles.trackWidth = str2double(get(hObject, 'String'));
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


function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.radius = str2double(get(hObject, 'String'));
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

handles.weightDistribution = str2double(get(hObject, 'String'));
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
