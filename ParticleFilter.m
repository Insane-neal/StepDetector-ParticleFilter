classdef ParticleFilter
    properties
        NumParticles;
        InitializationArea;
        Orientation;
        OrientationTimeStamp;
        StepLength;
        StepTimeStamp;
        SenseNoise;
        TurnNoise;
        ForwardNoise;
        Particles;
        Weights;
        Map;
        Meter2Pixel;
    end
    
    methods
        function obj = ParticleFilter(NumParticles,InitializationArea,Map,Meter2Pixel)
            obj.NumParticles = NumParticles;
            obj.InitializationArea = InitializationArea;
            obj.Map=Map;
            obj.Meter2Pixel=Meter2Pixel;
        end
        
        function obj = Initialization(obj,InitX,InitY,InitOrientation)
            for i=1:obj.NumParticles
                ParticleTemp=Particle(obj.InitializationArea,InitX,InitY,InitOrientation);
                obj.Particles{end+1}=ParticleTemp;
                obj.Weights(i)=1;
            end
        end
        
        function Particles = Move(obj,Turn,Forward)
            Particles=cell(obj.NumParticles,1);
            for i=1:obj.NumParticles
                Particles{i}=obj.Particles{i}.Move(Turn,Forward);
            end
        end
        
        function obj=SetTurnNoise(obj,NewTnoise)
            for i=1:obj.NumParticles
                obj.Particles{i}=obj.Particles{i}.SetTurnNoise(NewTnoise);
            end
        end
        
        function obj=SetForwardNoise(obj,NewFnoise)
            for i=1:obj.NumParticles
                obj.Particles{i}=obj.Particles{i}.SetForwardNoise(NewFnoise);
            end
        end
        
        function obj=SetSenseNoise(obj,NewTnoise)
            for i=1:obj.NumParticles
                obj.Particles{i}=obj.Particles{i}.SetSenseNoise(NewTnoise);
            end
        end
        
        function obj = Prediction(obj,OrienChange,StepLength)
            NextParticles=obj.Move(OrienChange,StepLength);     
            for i=1:obj.NumParticles
                CurrentPixel_x = round(obj.Particles{i}.x * obj.Meter2Pixel);
                CurrentPixel_y = round(obj.Particles{i}.y * obj.Meter2Pixel);
                NextPixel_x=round(NextParticles{i}.x*obj.Meter2Pixel);
                NextPixel_y=round(NextParticles{i}.y*obj.Meter2Pixel); 
                obj.Weights(i)=obj.MeasureMapProb(CurrentPixel_x,...
                    CurrentPixel_y,NextPixel_x,NextPixel_y);
            end
            norm=sum(obj.Weights);
            obj.Weights=obj.Weights./norm; %normalized weight array
            mW=max(obj.Weights);
            beta=0;
            
            index=randi(obj.NumParticles);
            for i=1:obj.NumParticles
                beta=beta+rand(1)*2*mW;
                while(beta>obj.Weights(index))
                    beta=beta-obj.Weights(index);
                    if(index==obj.NumParticles)
                        index=1;
                    end
                    index=index+1;                    
                end
                obj.Particles(i)=NextParticles(index);
            end
        end
        
        function  MapProb = MeasureMapProb(obj,CurrentPixel_x,CurrentPixel_y,NextPixel_x,NextPixel_y)
            MapProb=1;
            [length,width,~]=size(obj.Map);
            if(NextPixel_x<=0||NextPixel_x>=length||NextPixel_y<=0||NextPixel_y>=width)
                MapProb=0;
                return;
            end
            if((NextPixel_x==CurrentPixel_x)&&(NextPixel_y==CurrentPixel_y))
                MapProb=1;
                return;
            end
            %%
            Pixel_x=CurrentPixel_x;
            Pixel_y=CurrentPixel_y;
            if(abs(NextPixel_x-CurrentPixel_x)>abs(NextPixel_y-CurrentPixel_y))
                while(Pixel_x~=NextPixel_x)
                    pixel=obj.Map(round(Pixel_y),round(Pixel_x));
                    if(pixel~=255)
                        MapProb=0;
                        return; 
                    end
                    OneStep=(NextPixel_y-CurrentPixel_y)/(NextPixel_x-CurrentPixel_x);
                    if(CurrentPixel_x<NextPixel_x)
                        Pixel_x=Pixel_x+1;
                        Pixel_y=Pixel_y+OneStep;
                    elseif(CurrentPixel_x>NextPixel_x)
                        Pixel_x=Pixel_x-1;
                        Pixel_y=Pixel_y-OneStep;
                    else
                        return;
                    end
                end
            else
                while(Pixel_y~=NextPixel_y)
                    pixel=obj.Map(round(Pixel_y),round(Pixel_x));
                    if(pixel<=128)
                        MapProb=0;
                        return;
                    end
                    OneStep=(NextPixel_x-CurrentPixel_x)/(NextPixel_y-CurrentPixel_y);
                    if(CurrentPixel_y<NextPixel_y)
                        Pixel_x=Pixel_x+OneStep;
                        Pixel_y=Pixel_y+1;
                    elseif(CurrentPixel_y>NextPixel_y)
                        Pixel_x=Pixel_x-OneStep;
                        Pixel_y=Pixel_y-1;
                    else
                        return;
                    end
                end
            end
        end
    end
    methods(Static)
        
    end
end