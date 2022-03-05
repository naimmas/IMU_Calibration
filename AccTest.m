clc;clear;
fs = 100;
firstLoopNumSamples = fs*4;
secondLoopNumSamples = fs*4;
thridLoopNumSamples = fs*4;
totalNumSamples = firstLoopNumSamples + secondLoopNumSamples + thridLoopNumSamples;

traj = kinematicTrajectory('SampleRate',fs);

accBody = zeros(totalNumSamples,3);
angVelBody = zeros(totalNumSamples,3);
angVelBody(1:firstLoopNumSamples,1)     = (2*pi)/4;
angVelBody(firstLoopNumSamples+30:firstLoopNumSamples+secondLoopNumSamples ,2) = (2*pi)/4;
angVelBody(firstLoopNumSamples+secondLoopNumSamples+30:end,3) = (2*pi)/4;

[~,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);

IMU = imuSensor('accel-mag','SampleRate',fs);
IMU.Accelerometer = accelparams( ...
    'MeasurementRange',19.62, ...
    'Resolution',0.00059875, ...
    'ConstantBias',0.4905, ...
    'AxesMisalignment',2, ...
    'NoiseDensity',0.003924, ...
    'BiasInstability',0.0065, ...
    'TemperatureBias', [0.34335 0.34335 0.5886], ...
    'TemperatureScaleFactor',0.02);
[accelReadings] = IMU(accNED,angVelNED,orientationNED);

figure(1)
t = (0:(totalNumSamples-1))/fs;
plot(t,accelReadings)
legend('X-axis','Y-axis','Z-axis')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Readings')
save('AccTest.txt', 'accelReadings', '-ASCII','-append');