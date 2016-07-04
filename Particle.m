classdef Particle
    properties
        x;
        y;
        orientation;
        forwardNoise;
        turnNoise;
        senseNoise;
    end
    
    methods
        function obj=Particle(InitializationArea,Init_x,Init_y,Init_orientation)
            if(nargin>0)
                obj.x=(2*rand()-1)*InitializationArea+Init_x;
                obj.y=(2*rand()-1)*InitializationArea+Init_y;
                obj.orientation=Init_orientation;
                obj.forwardNoise=0;
                obj.turnNoise=0;
                obj.senseNoise=0;
            end
        end
        function obj=Set(obj,newX,newY,newOrientation)
            obj.x=newX;
            obj.y=newY;
            obj.orientation=newOrientation;
        end
        
        function obj=SetNoise(obj,NewFnoise,NewTnoise,NewSnoise)
            obj.turnNoise=NewTnoise;
            obj.forwardNoise=NewFnoise;
            obj.senseNoise=NewSnoise;
        end
        
        function obj=SetTurnNoise(obj,NewTnoise)
            obj.turnNoise=NewTnoise;
        end
        
        function obj=SetForwardNoise(obj,NewFnoise)
            obj.forwardNoise=NewFnoise;
        end
        
        function obj=SetSenseNoise(obj,NewSnoise)
            obj.senseNoise=NewSnoise;
        end
        
        function Z=Sense(obj)
            
        end
        
        function obj= Move(obj,turn,forward)
            if(forward<0)
                display('Error, forward cannot be less than 0');
            end
            %orientationLocal=obj.orientation+(turn)+normrnd(0,obj.turnNoise);
            orientationLocal=obj.orientation+(turn)+normrnd(0,obj.turnNoise);           
            orientationLocal=mod(orientationLocal,2*pi);
            
            dist=(forward)+normrnd(0,obj.forwardNoise);
            
            deltaX=sin(orientationLocal)*dist;
            deltaY=cos(orientationLocal)*dist;
            X=obj.x+deltaX;
            Y=obj.y+deltaY;
            
            obj=obj.Set(X,Y,orientationLocal);
            obj=obj.SetNoise(obj.forwardNoise,obj.turnNoise,obj.senseNoise);
        end
        
       
        
        %function prob=measurementProb(obj,measurement_x,measurement_y,wallPossibility)
        function prob=measurementProb(obj,measurement_x,measurement_y)
            dist=sqrt((obj.x - measurement_x)^2 + (obj.y - measurement_y)^2)/50;
            prob=obj.Gaussian(0,obj.senseNoise,dist);
        end        
    end
    
    methods(Static)
         function ValueG=Gaussian(mu,sigma,x)
            k=0.5*(((x-mu)^2)/(sigma^2));
            ValueG= (exp(-k))/(sqrt(2*pi*sigma^2));
        end
    end
end







