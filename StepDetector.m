classdef StepDetector
    %STEPDETECTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        %NS2S = 1/1000000000.0;
    end
    
    properties
        WindowSize;
        LegLength;
        Threshold;
        PeakValues;
        PeakTimeStamps;
        PeakFinalVelocity;
        ValleyValues;
        ValleyTimeStamps;
        ValleyFinalVelocity;
        Height;
        StepLength;
        HalfWindowSize;
        StepPeakValues;
        StepPeakTimeStamps;
        StepValleyValues;
        StepValleyTimeStamps;
        HeightValleyToPeak;
        HeightPeakToValley;
        PeakValueIndexes;
        ValleyValueIndexes;
        StepPeakValueIndexes;
        StepValleyValueIndexes;
        Velocity;
    end
    
    properties(SetAccess = private, GetAccess = private)
        TempFinalVelocity;
        TempHeight;
    end
    
    methods
        function obj=StepDetector(WindowSize,LegLength,Threshold)
            obj.LegLength = LegLength;
            obj.WindowSize = WindowSize;
            obj.HalfWindowSize=WindowSize/2;
            obj.Threshold=Threshold;
            obj.PeakValues = [];
            obj.PeakTimeStamps =[];
            obj.PeakValueIndexes = [];
            obj.ValleyValues = [];
            obj.ValleyTimeStamps=[];
            obj.ValleyValueIndexes = [];
            obj.StepPeakValueIndexes = [];
            obj.StepPeakTimeStamps = [];
            obj.StepPeakValues = [];
            obj.StepValleyTimeStamps=[];
            obj.StepValleyValueIndexes=[];
            obj.StepValleyValues=[];
            obj.Height = [];
            obj.StepLength = [];
            obj.PeakFinalVelocity = [];
            obj.ValleyFinalVelocity = [];
            obj.HeightPeakToValley = [];
            obj.Velocity=[];
        end
        
        function StepLength = CalculateLengthFromHeight(obj,Height)
            StepLength = 2*sqrt(obj.LegLength^2-(obj.LegLength-Height)^2);
        end
        
        function obj = CalculateHeightFromArray(obj,TimeArray, ProjectionArray)
            StartValleyToPeak = false;
            TempHeight = 0;
            HeightPeakToValley = 0;
            HeightValleyToPeak = 0;
            Velocity = 0;
            PrevVelocity = 0;
            
            IsFirstValue=true;
            for i=2:length(TimeArray)
                DeltaT = (TimeArray(i)-TimeArray(i-1));
                Velocity = ProjectionArray(i-1)*DeltaT+PrevVelocity;
                PrevHeight = TempHeight;
                
                if(StartValleyToPeak==false)
                    TempHeight = PrevHeight + Velocity*DeltaT + ...
                        0.5*ProjectionArray(i-1)*DeltaT^2;
                    %Use velocity cross zero as the transition condition
                    if(PrevVelocity*Velocity<=0&&~IsFirstValue)
                        obj.TempFinalVelocity=PrevVelocity;
                        obj.HeightPeakToValley(end+1)=PrevHeight;
                        StartValleyToPeak=true;
                        TempHeight=0;
                    end
                    if(IsFirstValue)
                        IsFirstValue=false;
                    end
                else
                    TempHeight = PrevHeight + Velocity*DeltaT + ...
                        0.5*ProjectionArray(i-1)*DeltaT^2;
                    if(i==length(TimeArray))
                        obj.HeightValleyToPeak(end+1)=TempHeight;
                    end
                end
                obj.Velocity=[obj.Velocity;Velocity];
                PrevVelocity=Velocity;
            end
            obj.TempFinalVelocity=Velocity;
            obj.Height(end+1)=obj.HeightPeakToValley(end)-obj.HeightValleyToPeak(end);
        end
        
        function obj=StepLengthCalculation(obj,VerticalAccelerationArray,TimeStampArray)
            obj=PeakValleyValuesDetection(obj,VerticalAccelerationArray,TimeStampArray);
            for i=1:length(obj.PeakValueIndexes)-1
                if(VerticalAccelerationArray(obj.PeakValueIndexes(i))>obj.Threshold...
                        &&VerticalAccelerationArray(obj.ValleyValueIndexes(i+1))<-obj.Threshold)
                    obj.StepPeakValueIndexes(end+1,1)=obj.PeakValueIndexes(i);
                    obj.StepValleyValueIndexes(end+1,1)=obj.ValleyValueIndexes(i+1);
                end
            end
            for i=1:length(obj.StepPeakValueIndexes)
                obj.StepPeakTimeStamps=[obj.StepPeakTimeStamps;TimeStampArray(obj.StepPeakValueIndexes(i))];
                obj.StepPeakValues=[obj.StepPeakValues;VerticalAccelerationArray(obj.StepPeakValueIndexes(i))];
                obj.StepValleyTimeStamps=[obj.StepValleyTimeStamps;TimeStampArray(obj.StepValleyValueIndexes(i))];
                obj.StepValleyValues=[obj.StepValleyValues;VerticalAccelerationArray(obj.StepValleyValueIndexes(i))];
            end
            for i=1:length(obj.StepPeakValueIndexes)-1
                %Use the zero velocity as the end condition. Use one
                %period of values as input to calculate the height.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                AccelerationPeakToPeak=VerticalAccelerationArray(obj.StepPeakValueIndexes(i):obj.StepPeakValueIndexes(i+1));
                TimePeakToPeak=TimeStampArray(obj.StepPeakValueIndexes(i):obj.StepPeakValueIndexes(i+1));
                obj=CalculateHeightFromArray(obj,TimePeakToPeak,AccelerationPeakToPeak);
                obj.PeakFinalVelocity=[obj.PeakFinalVelocity;obj.TempFinalVelocity];
                obj.HeightPeakToValley=[obj.HeightPeakToValley;obj.TempHeight];
                
                %                 AccelerationValleyToValley=VerticalAccelerationArray(obj.StepValleyValueIndexes(i):obj.StepValleyValueIndexes(i+1));
                %                 TimeValleyToValley=TimeStampArray(obj.StepValleyValueIndexes(i):obj.StepValleyValueIndexes(i+1));
                %                 obj=CalculateHeightFromArray(obj,TimeValleyToValley,AccelerationValleyToValley);
                %                 obj.ValleyFinalVelocity=[obj.ValleyFinalVelocity;obj.TempFinalVelocity];
                %                 obj.HeightValleyToPeak = [obj.HeightValleyToPeak;obj.TempHeight];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Use extreme value as the end condition. Use half period of
                %values as input to calculate the height.
                %                 AccelerationPeakToValley= VerticalAccelerationArray(obj.StepPeakValueIndexes(i):obj.StepValleyValueIndexes(i));
                %                 TimePeakToValley=TimeStampArray(obj.StepPeakValueIndexes(i):obj.StepValleyValueIndexes(i));
                %                 obj=CalculateHeightFromArray(obj,TimePeakToValley,AccelerationPeakToValley);
                %                 obj.PeakFinalVelocity=[obj.PeakFinalVelocity;obj.TempFinalVelocity];
                %                 obj.HeightPeakToValley=[obj.HeightPeakToValley;obj.TempHeight];
                %
                %                 AccelerationValleyToPeak=VerticalAccelerationArray(obj.StepValleyValueIndexes(i):obj.StepPeakValueIndexes(i+1));
                %                 TimeValleyToPeak=TimeStampArray(obj.StepValleyValueIndexes(i):obj.StepPeakValueIndexes(i+1));
                %                 obj=CalculateHeightFromArray(obj,TimeValleyToPeak,AccelerationValleyToPeak);
                %                 obj.ValleyFinalVelocity=[obj.ValleyFinalVelocity;obj.TempFinalVelocity];
                %                 obj.HeightValleyToPeak = [obj.HeightValleyToPeak;obj.TempHeight];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                StepLengthTemp=CalculateLengthFromHeight(obj,obj.Height(end));
                obj.StepLength=[obj.StepLength;StepLengthTemp];
            end
        end
        
        function obj=PeakValleyValuesDetection(obj,verticleAccelerationArray, timeStampArray)
            for i=obj.HalfWindowSize+1:length(verticleAccelerationArray)-obj.HalfWindowSize
                CurrentValue=verticleAccelerationArray(i);
                CurrentWindow=verticleAccelerationArray(i-obj.HalfWindowSize:i+obj.HalfWindowSize);
                MaxValue=max(CurrentWindow);
                MinValue=min(CurrentWindow);
                if(MaxValue==CurrentValue)
                    obj.PeakValues=[obj.PeakValues;CurrentValue];
                    obj.PeakTimeStamps=[obj.PeakTimeStamps;timeStampArray(i)];
                    obj.PeakValueIndexes=[obj.PeakValueIndexes;i];
                end
                if(MinValue==CurrentValue)
                    obj.ValleyValues=[obj.ValleyValues;CurrentValue];
                    obj.ValleyTimeStamps=[obj.ValleyTimeStamps;timeStampArray(i)];
                    obj.ValleyValueIndexes=[obj.ValleyValueIndexes;i];
                end
            end
        end
    end
    methods(Static)
        
    end
end

