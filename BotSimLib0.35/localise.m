function [bottemp] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =1200; % number of particles
particles(num,1) = BotSimCCMZ2; %how to set up a vector of objects

samples = 18; %number of samples per scan
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
maxNumOfIterations = 5;
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
            diff = sum( norm((botScan)  -( PartScan(:,i)) ,2) );           % define difference
            weight(i) = (1 / sqrt_r / sqrt_2pi) * exp(-(diff)^2 / 2 / r);
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

       for i = 1:num                        
           threshold = rand;
           wtempsum = 0;
           for j = 1:num
             wtempsum = wtempsum + weight(j);
             if wtempsum >= threshold
                 particles(i).setBotPosAng( particles(j).getBotPosAng() );
                 break;
             end
           end
           ParticlesPos(:,i) = particles(i).getBotPosAng();
       end
          
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
     Pcenter(:,(n+1)) = sum(ParticlesPos,2) / num; 

    %% Write code to check for convergence    
if  ( var(weight(:))  > 0 ) 
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
else
    converged = 0;
    n
end  % end if converaged

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
