%% init

clc;
clear all;
close all;

lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;

% Data Byte Length
LEN_MX_GOAL_POSITION        = 2;
LEN_MX_PRESENT_POSITION     = 2;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID                     = 18;            % Dynamixel#1 ID: 1
DXL2_ID                     = 17;            % Dynamixel#2 ID: 2
DXL3_ID                     = 15;            % Dynamixel#2 ID: 3
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM12';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 480;          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 600;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite instance
group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

index = 1;
dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_addparam_result = false;                % AddParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl1_present_position = 0;                  % Present position
dxl2_present_position = 0;

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end



initEnableMotor(port_num,PROTOCOL_VERSION,DXL1_ID,ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

initEnableMotor(port_num,PROTOCOL_VERSION,DXL2_ID,ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

initEnableMotor(port_num,PROTOCOL_VERSION,DXL3_ID,ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

%% video process calculation & write
disp(imaqhwinfo);

info = imaqhwinfo('winvideo',2)
disp([info.SupportedFormats]);


obj = videoinput('winvideo',2,'YUY2_640x480');
h = preview(obj);
h2=figure(2); %新建显示图像figure,同时获取句柄
triggerconfig(obj,'manual');
start(obj);
i=0;
se = strel('cube',16);

X = 320;
Y = 240;
untrustedX = 320;
untrustedY = 240;
preX = 320;
preY = 240;
XforArm = 0;
YforArm = 125;
ZforArm = 30;
Trust = 3;
while i<=8
    frame=getsnapshot(obj); %捕获图像
    frame=ycbcr2rgb(frame); %转成彩色,这个frame就可以按照自己意愿处理了
    subplot(2,3,1),imshow(frame);
    % imshow(noisyI);
    
    HSV=rgb2hsv(frame);
    erodedBW = imerode(HSV(:,:,1), se);
%     subplot(2,3,2),imshow(erodedBW); %显示图像
    image1=im2bw(erodedBW,0.7);
    
    subplot(2,3,2),imshow(image1); %显示图像
    
    
    
    % clc; clear all; close all;
    % I = imread('rice.png');
    % BW = im2bw(I, graythresh(I));
    [B,L] = bwboundaries(image1,'noholes');
    subplot(2,3,3),imshow(label2rgb(L, @jet, [.5 .5 .5]))
    hold on
    for k = 1:length(B)
        boundary = B{k};
        boundary(:,1) = 480 - boundary(:,1);
        subplot(2,3,4),plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);
        axis([0 640 0 480]);
        untrustedX = mean(boundary(:,2));
        untrustedY = mean(boundary(:,1));
    end
    
    if (untrustedX - X > 10 || untrustedY - Y >10)
        Trust = Trust - 1;
    else
        Trust = Trust + 1;
        X = untrustedX;
        Y = untrustedY;
    end
    if (Trust < 0)
        Trust = 0;
    end
    if (Trust > 3)
        Trust = 3;
    end
    if Trust == 0
        X = untrustedX;
        Y = untrustedY;
    end
        
    XforArm = -155+0.790816.*(X-148);
    YforArm = 125+0.797872.*(Y-30);
    ZforArm = 90;
    subplot(2,3,5),scatter(X,Y,'r','filled');
    axis([0 640 0 480]);
    subplot(2,3,6),scatter(XforArm,YforArm,'r','filled');
    axis([-160 160 100 300]);
    disp([i,X,Y,XforArm,YforArm,Trust])
    % frame=rgb2gray(frame);
    % tt=graythresh(frame);
    % image1=im2bw(frame,tt);
    % imshow(image1); %显示图像
    drawnow; % 实时更新图像
    i=i+1;
end


YforArm = YforArm+15;
XforArm = XforArm-8;
delete(obj);

% go above block

coor = [XforArm YforArm ZforArm + 50];
thetasDeg = solve_angle(coor);
thetas = thetasDeg./3.1415926.*180;
wd = zeros(3,1);
wd(1) = 512 - thetas(1).*1024./300;
wd(2) = 512 + thetas(2).*1024./300;
wd(3) = 512 + 68 - thetas(3).*1024./300;

% velicity trajectory
dxl_present_position=zeros(3,1);
dxl_present_position(1) = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_PRESENT_POSITION);%Read Dynamixel#1 present position
dxl_present_position(2) = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_PRESENT_POSITION);
dxl_present_position(3) = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_PRESENT_POSITION);
wpts=[dxl_present_position(1) wd(1)
      dxl_present_position(2) wd(2)
      dxl_present_position(3) wd(3)];
 tps=0:1;
 tvec=0:0.01:1;
 clear q;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tps, tvec);
siz=size(q);
path=siz(2);


for i= 1:path
    motorWriteStr(group_num,DXL1_ID,q(1,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL2_ID,q(2,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL3_ID,q(3,i),LEN_MX_GOAL_POSITION);
    allWriteSync(group_num,port_num,PROTOCOL_VERSION);
    pause(0.01);
%     fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n\n', DXL1_ID, wd(1), dxl1_present_position, DXL2_ID, wd(2), dxl2_present_position, DXL3_ID, wd(3), dxl3_present_position);
    
%     if ~((abs(wd(1) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(2) - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(3) - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD))
%          fprintf("Frequence too fast\n");
%         break;
%     end
end

% onto the block
exwd = wd;

coor = [XforArm YforArm ZforArm];
thetasDeg = solve_angle(coor);
thetas = thetasDeg./3.1415926.*180;
wd = zeros(3,1);
wd(1) = 512 - thetas(1).*1024./300;
wd(2) = 512 + thetas(2).*1024./300;
wd(3) = 512 + 68 - thetas(3).*1024./300;

motorWriteStr(group_num,DXL1_ID,wd(1),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL2_ID,wd(2),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL3_ID,wd(3),LEN_MX_GOAL_POSITION);
allWriteSync(group_num,port_num,PROTOCOL_VERSION);

% velicity trajectory
dxl_present_position=exwd;
wpts=[dxl_present_position(1) wd(1)
      dxl_present_position(2) wd(2)
      dxl_present_position(3) wd(3)];
 tps=0:1;
 tvec=0:0.01:1;
 clear q;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tps, tvec);
siz=size(q);
path=siz(2);


for i= 1:path
    motorWriteStr(group_num,DXL1_ID,q(1,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL2_ID,q(2,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL3_ID,q(3,i),LEN_MX_GOAL_POSITION);
    allWriteSync(group_num,port_num,PROTOCOL_VERSION);
    pause(0.01);
%     fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n\n', DXL1_ID, wd(1), dxl1_present_position, DXL2_ID, wd(2), dxl2_present_position, DXL3_ID, wd(3), dxl3_present_position);
    
%     if ~((abs(wd(1) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(2) - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(3) - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD))
%          fprintf("Frequence too fast\n");
%         break;
%     end
end

% lift
pause(3);
exwd = wd;
coor = [XforArm YforArm ZforArm+80];
thetasDeg = solve_angle(coor);
thetas = thetasDeg./3.1415926.*180;
wd = zeros(3,1);
wd(1) = 512 - thetas(1).*1024./300;
wd(2) = 512 + thetas(2).*1024./300;
wd(3) = 512 + 68 - thetas(3).*1024./300;

motorWriteStr(group_num,DXL1_ID,wd(1),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL2_ID,wd(2),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL3_ID,wd(3),LEN_MX_GOAL_POSITION);
allWriteSync(group_num,port_num,PROTOCOL_VERSION);

% velicity trajectory
dxl_present_position=exwd;
wpts=[dxl_present_position(1) wd(1)
      dxl_present_position(2) wd(2)
      dxl_present_position(3) wd(3)];
 tps=0:1;
 tvec=0:0.01:1;
 clear q;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tps, tvec);
siz=size(q);
path=siz(2);

plot(q(3,i));

for i= 1:path
    motorWriteStr(group_num,DXL1_ID,q(1,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL2_ID,q(2,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL3_ID,q(3,i),LEN_MX_GOAL_POSITION);
    allWriteSync(group_num,port_num,PROTOCOL_VERSION);
    pause(0.01);
%     fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n\n', DXL1_ID, wd(1), dxl1_present_position, DXL2_ID, wd(2), dxl2_present_position, DXL3_ID, wd(3), dxl3_present_position);
    
%     if ~((abs(wd(1) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(2) - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(3) - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD))
%          fprintf("Frequence too fast\n");
%         break;
%     end
end

% one left punch
exwd = wd;
wd = [800 650 500];
motorWriteStr(group_num,DXL1_ID,wd(1),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL2_ID,wd(2),LEN_MX_GOAL_POSITION);
motorWriteStr(group_num,DXL3_ID,wd(3),LEN_MX_GOAL_POSITION);


allWriteSync(group_num,port_num,PROTOCOL_VERSION);

% velicity trajectory
dxl_present_position=exwd;
wpts=[dxl_present_position(1) wd(1)
      dxl_present_position(2) wd(2)
      dxl_present_position(3) wd(3)];
 tps=0:1;
 tvec=0:0.01:1;
 clear q;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tps, tvec);
siz=size(q);
path=siz(2);


for i= 1:path
    motorWriteStr(group_num,DXL1_ID,q(1,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL2_ID,q(2,i),LEN_MX_GOAL_POSITION);
    motorWriteStr(group_num,DXL3_ID,q(3,i),LEN_MX_GOAL_POSITION);
    allWriteSync(group_num,port_num,PROTOCOL_VERSION);
    pause(0.01);
%     fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n[ID:%03d] GoalPos:%03d  PresPos:%03d\n\n', DXL1_ID, wd(1), dxl1_present_position, DXL2_ID, wd(2), dxl2_present_position, DXL3_ID, wd(3), dxl3_present_position);
    
%     if ~((abs(wd(1) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(2) - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(wd(3) - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD))
%          fprintf("Frequence too fast\n");
%         break;
%     end
end


%% end
disableMotor(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
disableMotor(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
disableMotor(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

disp('Port closed successfully!!!');