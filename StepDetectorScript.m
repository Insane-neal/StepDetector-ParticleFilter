clear all;
close all;
WindowSize = 60;
LegLength=0.83;
Threshold=1;
load ('..\Data\IndoorLocation-2016-5-30-10-6-50\data.mat');
Map=imread('floor6_bw.jpg');
SD=StepDetector(WindowSize,LegLength,Threshold);
SD=SD.StepLengthCalculation(PROJECTION_ON_GRAVITY,PROJECTION_ON_GRAVITY_TIME);
ymin=-3;
ymax=3;
%plot([SD.PeakTimeStamps,SD.PeakTimeStamps],[ymin,ymax],'green');
plot([SD.StepPeakTimeStamps,SD.StepPeakTimeStamps],[ymin,ymax],'green');
hold on;
plot(PROJECTION_ON_GRAVITY_TIME,PROJECTION_ON_GRAVITY);
hold on;
scatter(SD.StepPeakTimeStamps,SD.StepLength*10);
%plot([SD.ValleyTimeStamps,SD.ValleyTimeStamps],[ymin,ymax],'red');

