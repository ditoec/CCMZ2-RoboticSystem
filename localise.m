function [bottemp] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSimCCMZ2; %how to set up a vector of objects

samples = 10; %number of samples per scan
ParticlesPos = zeros(3,num); %*****

for i = 1:num
    particles(i) = BotSimCCMZ2(modifiedMap,[1 0.1 0.05],samples);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    ParticlesPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    particles(i).drawBot(3); %draw particle with line length 3 and default color
end
        %figure(3);
        %hold off;
        %botSim.drawMap();
        %for i=1:num
        %    plot(ParticlesPos(1,i),ParticlesPos(2,i),'b.', 'markersize',15);
        %end
        %BotPos = botSim.getBotPos();
        %plot(BotPos(1),BotPos(2),'r.', 'markersize',25);
        %hold on;
        
%% Localisation code
maxNumOfIterations = 10;
n = 0;
converged =0; %The filter has not converged yet

r = 500; % variance of Gussan function
sqrt_r = sqrt(r); %precompute to speed up calculation
sqrt_2pi = sqrt(2*pi);

botSim.setScanConfig(botSim.generateScanConfig(samples)); 
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);
drawnow;
weight = zeros(num,1);
Pcenter = zeros(3,maxNumOfIterations);
PartScan = zeros(samples, num);

Pcenter(:,1) = sum(ParticlesPos,2) / num;

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    for i=1:num
        if particles(i).insideMap() == 1
        PartScan(:,i) = particles(i).ultraScan(); % initialise particles
    %% Write code for scoring your particles           
            diff = sum(norm((botScan)  -( PartScan(:,i)) ,2) );     %(norm((botScan)  -( PartScan(:,i)) ,2) );           % define difference
            weight(i) = (1 /sqrt_r /sqrt_2pi) * exp(-(diff)^2 / 2/r);
        else
            weight(i) = 0;
        end
    end
    
    weightsum = sum(weight);                % normalization
    for i = 1:num
        weight(i) = weight(i) / weightsum;
    end
    
    %% Write code for resampling your particles 
% The first way to re-sample
% Sequence re-sampling

%        for i = 1:num                        
%            threshold = rand;
%            wtempsum = 0;
%            for j = 1:num
%              wtempsum = wtempsum + weight(j);
%              if wtempsum >= threshold
%                  particles(i).setBotPosAng( particles(j).getBotPosAng() );
%                  break;
%              end
%            end
%            ParticlesPos(:,i) = particles(i).getBotPosAng();
%        end
          
% The sencond way to re-sample
% random re-sampling
    %for i = 1:num
    %    wmax = 2 * max(weight) * rand;  
    %    index = randi(num, 1);
    %    while( wmax > weight(index) )
    %        wmax = wmax - weight(index);
    %        index = index + 1;
    %        if index > num
    %            index = 1;
    %        end
    %    end
    %    particles(i).setBotPos( particles(index).getBotPos() );
    %    particles(i).setBotAng( particles(index).getBotAng() );
    %    ParticlesPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    %end
    
    
    
 %%% calculate Neff
%  
%     Neff=0;
%     for i=1:num
%     Neff= Neff + sqrt(weight(i));
%     end
%     Neff= 1/Neff;
%     if Neff < threshold
%  
    % The third way to re-sample
     c=zeros(num,1);
     u=zeros(num,1);
     c(1)= weight(1);
     m=1; % number of walls
     for i=2:num
         c(i)=c(i-1)+weight(i);
     end
     u(1)= (1/num);%.*rand(1,1);
     i=1;
    for j=2:num
     u(j)=u(1)+((j-1)/num);
    end
     %newParticlesPos = zeros(3,num); %*****
     for j=1:num
         
         l=i;
        
         
         if (i > num)
             break;
         end
         while (u(j) > c(i))
            if (m~=1)
                %generate particle here
                %[Xk,Wk] Gen(particle(i),weight(i),m)
                s= zeros(1,m);
                k=1;
               sigma=zeros(1,fix(m/2));
                for a=1: fix(m/2)
                   sigma(a)= 1/(2*a);
                    x=((m-2*a)/(4*num))* sigma(a);  % {((m-2*n)/(4*N)) * sigma} %there should be a onstant value sigma     
                    s(k:k+1)=[-x,x];
                    k=k+2;
                end
        
                for b= 0:m-1
                    particles(j-b).setBotPos([ParticlesPos(1,i) ParticlesPos(2,i)]+s(b+1));
                    particles(j-b).setBotAng(ParticlesPos(3,i));
                    
                end
                    m=1;
            end
            if (i == num)
            break;
            end
            i=i+1;
         end
         if (i-l)>1
            %[Xk,Wk] MAX(wieght(l+1),weight(i))
            [w,index]=max(weight(l+1:i));
            particles(j).setBotPos( [ParticlesPos(1,l+index) ParticlesPos(2,l+index)]);
            particles(j).setBotAng( ParticlesPos(3,l+index)); 
         elseif (i-l)==1
            %[Xk,Wk] particle(i), weight(i)
            particles(j).setBotPos([ParticlesPos(1,i) ParticlesPos(2,i)]);
            particles(j).setBotAng(ParticlesPos(3,i));              
         else
             if l==1
                %[Xk,Wk] particle(i), weight(i)
              particles(j).setBotPos([ParticlesPos(1,i) ParticlesPos(2,i)]);
              particles(j).setBotAng(ParticlesPos(3,i));         
             else
                m=m+1;
             end
             if j==num
                %[Xk,Wk] Gen (particle(i), weight(i),m)
                s= zeros(1,m);
                k=1;
                sigma=zeros(1,fix(m/2));
                for a=1: fix(m/2)
                    sigma(a)= 1/(2*a);
                    x=((m-2*a)/(4*num)) *sigma(a);  % {((m-2*n)/(4*N)) * sigma} %there should be a onstant value sigma     
                    s(k:k+1)=[-x,x];
                    k=k+2;
                end

                for b= 0:m-1
                particles(j-b).setBotPos([ParticlesPos(1,i) ParticlesPos(2,i)]+s(b+1));
                particles(j-b).setBotAng( ParticlesPos(3,i));
                
                end
             end
         end   
     end
for i = 1:num
    ParticlesPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    %particles(i).drawBot(3); %draw particle with line length 3 and default color 
end
    
    
     %Pcenter(:,(n+1)) = sum(ParticlesPos,2) / num; 

    %% Write code to check for convergence    
%          if ( distance(botSim.getBotPos(),[Pcenter(1,n+1) Pcenter(2,n+1)]) <3 )
%              break;       % OR converged=1;
%          end
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
     perc= fix(num * .2); % respawn 20% of particles
     randIndeces=randi([1,num],1,perc);
     for i=1:perc
         particles(randIndeces(i)).randomPose(0);
     end
    
     for i = 1:num
     ParticlesPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
     end
 if  ( std(weight(:))  > 0 ) 
    %% Write code to decide how to move next
%    here they just turn in cicles as an example
     [m, i] = max(botScan);       % move to longest orientation
     turn = (i-1)/samples * 2 * pi;
    %turn = pi/8;
    move = 5;

    %botSim.setMotionNoise(0.1);
    %botSim.setTurningNoise( 0.05 );
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        %particles(i).setMotionNoise( randn(1,1) ); %/*****
        %particles(i).setTurningNoise( 0.05 );
        particles(i).turn_move(turn,move); %turn & move the particle in the same way as the real robot
        %particles(i).move(move); %move the particle in the same way as the real robot
        ParticlesPos(:,i) = particles(i).getBotPosAng();
    end
    Pcenter(:,n) = sum(ParticlesPos,2) / num; 
    
end 

    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        figure(1);
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        BotPos = botSim.getBotPos();
        BotAng = botSim.getBotAng();
        plot(BotPos(1),BotPos(2),'r.', 'markersize',25);
        text(65,10,...
        ['RealBot:(',num2str(BotPos(1)),',',num2str(BotPos(2)),',',num2str(mod(BotAng,2*pi)/pi*180),')'])
        drawnow;
        plot(Pcenter(1, n), Pcenter(2, n), 'k.', 'markersize',25);
        text(65, 5,...
        ['EstBot:(',num2str(Pcenter(1, n)),',',num2str(Pcenter(2, n)),',',num2str(Pcenter(3, n)/pi*180),')'])
        %plot(target(1), target(2), 'g.', 'markersize',25);
        %text(target(1)+5, target(2)+5,...
        %['Target:(',num2str(target(1)),',',num2str(target(2)),')'])        
        drawnow;
    end
end
%% find initial angle
bottemp = BotSim(modifiedMap,[0,0,0]); 
bottemp.setBotPos(Pcenter(1:2, n)');
bottemp.setBotAng( Pcenter(3, n));
%bottemp.setScanConfig(bottemp.generateScanConfig(samples));
%tempDiff = zeros(360,1);
%for j = 0:360
%    bottemp.setBotAng(j/180*pi);
%    tempScan = bottemp.ultraScan();
%    tempDiff(j+1) = sum( abs(botScan - tempScan) );
%end
%[~, Ang]=min(tempDiff);
%figure(6)
%plot(tempDiff)
  %text(Pcenter(1, n)+5, Pcenter(2, n),...
   %     ['BotAngle = ',num2str(mod(botSim.getBotAng(),2*pi)*180/pi)])

end
