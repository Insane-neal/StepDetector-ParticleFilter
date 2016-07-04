function []=MyParticleFilter(fwd_Noise,turn_Noise,sense_Noise,orientation,steps,predicted_Label,no_Particles,map_Info,actual_Label,stepLength)
%set(0,'DefaultFigureWindowStyle','docked')
map_Info=[map_Info(:,1),map_Info(:,2)-730,map_Info(:,3)+1630];
%map_Info=[map_Info(:,1),map_Info(:,3)+1630,map_Info(:,2)-730];
%constant
no_Iterations=length(steps)-1;
%stepLength=0.85;
initializationArea=30;

%mapping between label and coordinates for initialization
I=find(map_Info(:,1)==predicted_Label(1));
init_x=map_Info(I,2);
init_y=map_Info(I,3);


%myRobot=Myrobot(initializationArea,init_x,init_y);
C=[1,0,0;0,1,0;0,0,1];
N=no_Particles;

for i=1:N
    particle=Myrobot(initializationArea,init_x,init_y,orientation(1)/180*pi);
    particle=particle.SetNoise(fwd_Noise,turn_Noise,sense_Noise);
    P(i)=particle;
    clear x;
end

for i=1:N
    X(i)=P(i).x;
    Y(i)=P(i).y;
    O(i)=P(i).orientation;
end

%disp(myRobot.eval(P));%remains to be done
%Loadimage

for k=1:no_Iterations%total poses-1
    fprintf('%d',orientation(k));
    I=find(map_Info(:,1)==predicted_Label(k));
    measurement_x=map_Info(I,2);
    measurement_y=map_Info(I,3);
    J=find(map_Info(:,1)==actual_Label(k));
    actual_x=map_Info(J,2);
    actual_y=map_Info(J,3);
%     K=find(map_Info(:,1)==predicted_Label(k));
%     predicted_x=map_Info(K,2);
%     predicted_y=map_Info(K,3);
    %hold off;
    %Loadimage
    
    map = imread('testWallRestriction3.jpg');    
    msize=size(map);
    %scatter(X,-Y,3,C(1,:));
    fprintf('%d',k);
    %hold on;
%     if k>1
%     for i=1:N
%         A=[Pcurrent(i).x -Pcurrent(i).y];
%         B=[P(i).x -P(i).y];
%         line(A,B);
%     end
%     end
    %figure(1)   
    x0=sum(X)/no_Particles;
    y0=sum(-Y)/no_Particles;    
    r=(max(X)-min(X)+max(Y)-min(Y))/4;
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
%%%%%%%%%%%%%%%%%%plot figure 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure(1);
%     image(map); 
%     title('CEI Second Floor');    
%     hold on; 
%     plot(x0,y0,'--rs','markersize',10,'MarkerFaceColor','r');
%     plot(x0+xp,y0+yp,'r');
%     plot(actual_x,-actual_y,'.','markersize',20);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure(2)
%     set(gca,'XLim',[0,440],'YLim',[-400,0]);  
%     hold on;     
%     plot(sum(X)/no_Particles,sum(Y)/no_Particles,'--ro','markersize',5,'MarkerFaceColor','r');    
    
   
%     set(gca,'XLim',[800,1200],'YLim',[-2000,-1600]); 
   
    
    hold on;
    figure(2);
    map2 = imread('map_new.png');
    image(map2); 
    title('CEI Second Floor');
    msize=size(map2);
    hold on;    
    plot(x0,y0,'--rs','markersize',10,'MarkerFaceColor','r');
    plot(x0+xp,y0+yp,'r');
    plot(actual_x,-actual_y,'.','markersize',20); 
    %scatter(X,-Y,3,C(1,:));
    hold on;
    
    %plot(predicted_x-720,-predicted_y-1630,'k--o');
    %plot(predicted_x,-predicted_y,'k--.','markersize',20); 
    drawnow;
    pause(0.3);
%     linkdata on;
    turn_Angle=(orientation(k+1)-orientation(k))/360*(2*pi);
    disp(k);
    disp(turn_Angle);    
    distance=stepLength*(steps(k+1)-steps(k))*15;%unit:pixel
    
    for i=1:N
        wallPossibility(i)=1;
        Pcurrent(i)=P(i);%save the current location        
        PcurrentX=ceil(Pcurrent(i).x);
        PcurrentY=-ceil(Pcurrent(i).y);        
        P(i)=P(i).Move(turn_Angle,distance);
        PnextX=ceil(P(i).x);
        PnextY=-ceil(P(i).y);
        if PnextX<0||PnextX>msize(2)||PnextY<0||PnextY>msize(1)
            wallPossibility(i)=0;
        end
        BooleanX=PcurrentX>PnextX;%estimate the moving direction
        BooleanY=PcurrentY>PnextY;
        
        if (abs(PcurrentX-PnextX)>1)||(abs(PcurrentY-PnextY)>1)
            subImage=map(max(min(PcurrentY,PnextY),1):min(max(PcurrentY,PnextY),msize(1)-1),max(min(PcurrentX,PnextX),1):min(max(PcurrentX,PnextX),msize(2)-1));
            L= bwlabel(round(subImage/256), 8);
            [l, w]=size(L);
            if l*w~=0
            if L(BooleanX*(l-1)+1,BooleanY*(w-1)+1)~=L(~BooleanX*(l-1)+1,~BooleanY*(w-1)+1)
                wallPossibility(i)=0;
            end
            end
        end
        
        
    end

    for i=1:N
        %W(i)=P(i).measurementProb(measurement_x,measurement_y,wallPossibility(i));
        W(i)=P(i).measurementProb(measurement_x,measurement_y);
    end

    norm=sum(W);
    W=W./norm; %normalized weight array

    mW=max(W);
    beta=0;
    %delete particles cross the wall
    particleIndex=[];
    for i =1:N
    	if wallPossibility(i)~=0
            particleIndex(length(particleIndex)+1)=i;
        end            
    end
    for i=1:length(particleIndex)
        Pwallrestriction(i)=P(particleIndex(i));
    end
    index=randi(length(particleIndex)-1);
    for i=1:N
        beta=beta+rand(1)*2*mW;
        while(beta>W(index))
            beta=beta-W(index);
            index=index+1;
            if(index==length(particleIndex))
                index=index-length(particleIndex)+1;
            end
        end             
        P1(i)=Pwallrestriction(index);        
    end
    P=P1;
    
    for i=1:N
    X(i)=P(i).x;
    Y(i)=P(i).y;
    O(i)=P(i).orientation;
    end
    
%    disp(myRobot.eval(P));
end

% for i=1:N
%     X(i)=P(i).x;
%     Y(i)=P(i).y;
%     O(i)=P(i).orientation;
% end
end
