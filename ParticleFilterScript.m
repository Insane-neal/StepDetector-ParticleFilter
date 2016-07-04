clear all;
close all;
dbstop if error;
%load ('..\Data\IndoorLocation-2016-6-12-10-36-29\data.mat');
load ('..\Data\DataSamsungS7\IndoorLocation-2016-6-24-16-35-22\data.mat');
%%%%%%%%%%%%%%%%%%%%Particle Filter Constructor%%%%%%%%%%
Map=imread('floor6_bw.jpg');
NumParticles=50;
InitializationArea=1;
Meter2Pixel=63;
PF=ParticleFilter(NumParticles,InitializationArea,Map,Meter2Pixel);
%%%%%%%%%%%%%%%%%%%%%Init Step Detector%%%%%%%%%%%%%%
ymax=2;
WindowSize = 60;
LegLength=0.83;
Threshold=1;
ForwardNoise=0.1;
TurnNoise=pi/72;
SenseNoise=2;%Standard deviation
SD=StepDetector(WindowSize,LegLength,Threshold);
SD=SD.StepLengthCalculation(PROJECTION_ON_GRAVITY,PROJECTION_ON_GRAVITY_TIME);
RAW_ORIENTATION=RAW_ORIENTATION;
%%%%%%%%%%%%%%%%%%%Particle Filter initialization%%%%%%%
InitXinPixel=2612;
InitYinPixel=1690;
InitX=InitXinPixel/Meter2Pixel;
InitY=InitYinPixel/Meter2Pixel;
InitOrien=RAW_ORIENTATION(1);
PF=PF.Initialization(InitX,InitY,InitOrien);
PF=PF.SetForwardNoise(ForwardNoise);
PF=PF.SetSenseNoise(SenseNoise);
PF=PF.SetTurnNoise(TurnNoise);
%%Find the according orientation from the step timestamp
NumIteration=length(SD.StepLength);
RawStepOrien=zeros(1,NumIteration);
PFHistory{1,1}=PF;
x=zeros(length(PFHistory),length(PFHistory{1}.Particles));
y=zeros(length(PFHistory),length(PFHistory{1}.Particles));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Orientation%%%%%%%%%%%%%%%%%%%%%%%%%%%
OrienChange=[];
LastOrien=InitOrien;
for i=1:NumIteration
    [T,Index]=min(abs(GAME_ROTATION_VECTOR_TIME-SD.StepPeakTimeStamps(i)));
    CurrentOrien=RAW_ORIENTATION(Index);
    OrienChange(end+1)=CurrentOrien-LastOrien;
    LastOrien=CurrentOrien;
end


RAW_X_GAME_RV=zeros(NumIteration+1,1);
RAW_Y_GAME_RV=zeros(NumIteration+1,1);
RAW_X_GAME_RV(1)=InitX;
RAW_Y_GAME_RV(1)=InitY;
raw_orien=0;
for i=1:NumIteration
    raw_orien(1,end+1)=raw_orien(1,end)+OrienChange(i);
    deltaX=sin(raw_orien(1,end))*SD.StepLength(i);
    deltaY=cos(raw_orien(1,end))*SD.StepLength(i);
    RAW_X_GAME_RV(i+1)=RAW_X_GAME_RV(i)-deltaX;
    RAW_Y_GAME_RV(i+1)=RAW_Y_GAME_RV(i)+deltaY;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%Rotation Vector%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ROTATION_VECTOR=[ROTATION_VECTOR_X,ROTATION_VECTOR_Y,ROTATION_VECTOR_Z,ROTATION_VECTOR_W];
RawOrienRV=quatern2euler(ROTATION_VECTOR);
OrienChangeRV=[];
LastRotVec=RawOrienRV(1,1);
tic
for i=1:NumIteration
    [T,Index]=min(abs(ROTATION_VECTOR_TIME-SD.StepPeakTimeStamps(i)));
    CurrentRotationVector=RawOrienRV(Index,1);
    OrienChangeRV(end+1)=CurrentRotationVector-LastRotVec;
    LastRotVec=CurrentRotationVector; 
end
Raw_X_RV=zeros(NumIteration+1,1);
Raw_Y_RV=zeros(NumIteration+1,1);
Raw_X_RV(1)=InitX;
Raw_Y_RV(1)=InitY;
OrienRV=0;
for i=1:NumIteration
    OrienRV(end+1)=OrienRV(end)+OrienChangeRV(i);
    deltaX=sin(OrienRV(end))*SD.StepLength(i);
    deltaY=cos(OrienRV(end))*SD.StepLength(i);
    Raw_X_RV(i+1)=Raw_X_RV(i)+deltaX;
    Raw_Y_RV(i+1)=Raw_Y_RV(i)+deltaY;
end
% for i=1:NumIteration
%     [T,Index]=min(abs(GAME_ROTATION_VECTOR_TIME-SD.StepPeakTimeStamps(i)));
%     CurrentOrien=RAW_ORIENTATION(Index);
%     OrienChange(end+1)=CurrentOrien-LastOrien;
%     PF=PF.Prediction(OrienChange(end),SD.StepLength(i));
%     LastOrien=CurrentOrien;
%     RawStepOrien(1+i)=CurrentOrien;
%     PFHistory{1,end+1}=PF;
%     for j=1:length(PFHistory{i}.Particles)
%         x(i,j)=PFHistory{i}.Particles{j}.x;
%         y(i,j)=PFHistory{i}.Particles{j}.y;
%     end
% end
%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:NumIteration
%     [T,Index]=min(abs(GAME_ROTATION_VECTOR_TIME-SD.StepPeakTimeStamps(i)));
%     CurrentOrien=RAW_ORIENTATION(Index);
%     OrienChange(end+1)=CurrentOrien-LastOrien;
%     PF=PF.Prediction(OrienChange(end),SD.StepLength(i));
%     LastOrien=CurrentOrien;
%     RawStepOrien(1+i)=CurrentOrien;
%     PFHistory{1,end+1}=PF;
%     for j=1:length(PFHistory{i}.Particles)
%         x(i,j)=PFHistory{i}.Particles{j}.x;
%         y(i,j)=PFHistory{i}.Particles{j}.y;
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Plot particles on map
% figure(1);
% DELAY=0.25;
% for i=1:NumIteration
%     clf;
%     imshow(Map);
%     hold on;
%     scatter(x(i,:)*Meter2Pixel,y(i,:)*Meter2Pixel,10);
%     pause(DELAY);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot analytical figures
figure(2);
plot([SD.StepPeakTimeStamps,SD.StepPeakTimeStamps],[-ymax,ymax],'green');
hold on;
plot(PROJECTION_ON_GRAVITY_TIME,PROJECTION_ON_GRAVITY);
hold on;
scatter(SD.StepPeakTimeStamps,SD.StepPeakValues);
hold on;
scatter(SD.StepValleyTimeStamps,SD.StepValleyValues);
hold on;
%scatter(SD.StepPeakTimeStamps,SD.StepLength*10);
mean(SD.StepLength)
sum(SD.StepLength)
%%%%%%%%%%%%%%%
figure(3);
imshow(Map);
hold on;
plot(RAW_X_GAME_RV*Meter2Pixel,RAW_Y_GAME_RV*Meter2Pixel);
hold on;
scatter(RAW_X_GAME_RV*Meter2Pixel,RAW_Y_GAME_RV*Meter2Pixel);
figure(4);
imshow(Map);
hold on;
plot(Raw_X_RV*Meter2Pixel,Raw_Y_RV*Meter2Pixel);
hold on;
scatter(Raw_X_RV*Meter2Pixel,Raw_Y_RV*Meter2Pixel);