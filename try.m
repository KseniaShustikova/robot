clear all
close all

%% Start
ball = EV3();
ball.connect('usb');
%% Main

% Connect to the EV3 brick
modus='Angular';
%modus='Rate';

% Initialize the gyro sensors and motors
gyrosensor1 = ball.sensor1;
gyrosensor2 = ball.sensor2;
if strcmp(modus,'Angular')
    gyrosensor1.mode = DeviceMode.Gyro.Angular;
    gyrosensor2.mode = DeviceMode.Gyro.Angular;
elseif strcmp(modus,'Rate')
    gyrosensor1.mode = DeviceMode.Gyro.Rate;
    gyrosensor2.mode = DeviceMode.Gyro.Rate;
end
motorA = ball.motorA;
motorB = ball.motorB;
desiredAngle1 = gyrosensor1.value;
desiredAngle2 = gyrosensor2.value;

%PID variables
previousError1 = 0;
previousError2 = 0;
integral1 = 0;
integral2 = 0;
dt = 0.1; %time step
previousTime = tic;
totalTime = 3;
Kp = 1.5; %basic reaction to the error
Ki = 0.4; %eliminating steady errors, careful to avoid windup
Kd = 0.3; %smoothes 
currentAngle1 = 0;
currentAngle2 = 0;
n=0;
%motorA.start();
%motorB.start();

%gyrosensor1.reset();
%gyrosensor2.reset();


%FOR LOOP
%for t = dt:dt:totalTime
while 1
    elapsedTime = toc(previousTime);
    previousTime = tic;
    n=n+1;
    % Rate messen (funktioniert nicht so gut, weil die Werte auf Dauer weglaufen)
    if strcmp(modus,'Rate')
        rate1 = gyrosensor1.value;
        rate2 = gyrosensor2.value;
        %disp([num2str(rate1),' ',num2str(rate2)])
        currentAngle1 = currentAngle1 + rate1 * elapsedTime;
        currentAngle2 = currentAngle2 + rate2 * elapsedTime;
    elseif strcmp(modus,'Angular')
        currentAngle1 = gyrosensor1.value;
        currentAngle2 = gyrosensor2.value;
        %disp('Here')
    end
    currentAngles1(n)=currentAngle1;
    currentAngles2(n)=currentAngle2;
    %timepoints(n)=elapsedTime
    %disp([num2str(currentAngle1),' ',num2str(currentAngle2)])
    error1 = desiredAngle1 - currentAngle1;
    error2 = desiredAngle2 - currentAngle2;
    disp([num2str(error1),' ',num2str(error2)])
    integral1 = integral1 + error1 * elapsedTime;
    integral2 = integral2 + error2 * elapsedTime;
    derivative1 = (error1 - previousError1) / elapsedTime;
    derivative2 = (error2 - previousError2) / elapsedTime;
    %fron
    %{
    if rate1 > rate2 && rate1 > 0 && rate2 < 0
        motorPower = -(Kp * error + Ki * integral + Kd * derivative);
        motorA.power  = max(min(motorPower, 100), -100);
        motorB.power = 1;
    end
    %right
    if rate2 < rate1 && rate1 < 0 && rate2 < 0
        motorPower = -(Kp * error + Ki * integral + Kd * derivative);
        motorB.power = max(min(motorPower, 100), -100); 
        motorA.power = 1;
    end
    %back
    if rate1 < rate2 && rate1 < 0 && rate2 > 0
        motorPower = -(Kp * error + Ki * integral + Kd * derivative);
        motorA.power  = max(min(motorPower, 100), -100);
        motorB.power = 1;
    end
    %left
    if rate1 < rate2 && rate1 < 0 && rate2 > 0
        motorPower = (Kp * error + Ki * integral + Kd * derivative);
        motorB.power  = max(min(motorPower, 100), -100);
        motorA.power = 1;
    end
    %motorA.power  = max(min(motorPower, 100), -100);  
    %motorB.power = max(min(motorPower, 100), -100);  
    %}

    previousError1 = error1;
    previousError2 = error2;
    %pause(dt);
    %gyrosensor1.reset();
    %gyrosensor2.reset();
end
%%for n=1:100
% Read and display the rotation rate%%
% rotationrate = ball.sensor1.value; disp(['Gyro Rotation Rate: ', num2str(rotationrate), ' deg/s']);end
motorA.stop();
motorB.stop();

%% Break
motorA.stop();
motorB.stop();

ball.disconnect;