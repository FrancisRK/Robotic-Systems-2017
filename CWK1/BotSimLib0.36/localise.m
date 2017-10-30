function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%%% Assuming all maps have limsMin = [0 0] -- Could possibly change
limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 10; %sampling resouloution in cm
iterators = dims/res;
iterators = ceil(iterators)+[1 1]; %to counteract 1 based indexing
gridPoints = iterators + [1,1];
mapMatrix = zeros(gridPoints);
for i = 1:gridPoints(2)
    for j = 1:gridPoints(1)
        testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing
        %notice, that i and j have also been swapped here so that the only
        %thing left to do is invert the y axis. 
        mapMatrix(i,j) = botSim.pointInsideMap(testPos);
        if mapMatrix(i,j)
            plot(testPos(1),testPos(2),'*');%inside map
        else
            plot(testPos(1),testPos(2),'o');%outside map
        end
    end
end

mapGrid = flipud(mapMatrix);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%generate some random particles inside the map
num = 300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).drawBot(3);
end

botSim.drawMap();
%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged = 0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan(1,:) = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    
    %particleScans = cell(num, 1); %Creates a cell matrix to contain each particle's scans
    particleScans = zeros(num, 6);
    for i = 1:num
        particleScans(i,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles    
    
    particleDiff = zeros(num,6);
    averageParticleDiff = zeros(num,1);
    for i = 1:num
        particleDiff(i,:) = abs(botScan - particleScans(i,:)); %Calculates the absolute difference between particle and bot scans
        averageParticleDiff(i,:) = sum(particleDiff(i,:))/6; %Average difference between scans of bot and particle
    end
        
    particleWeight = zeros(num, 1);
    sigma = 1;
    for i = 1:num
        particleWeight(i,:) = (1/sqrt(2*pi*sigma))*exp(-((averageParticleDiff(i,:)^2)/2*sigma^2));
    end
        
    normalWeight = zeros(num, 1);
    for i = 1:num
        normalWeight(i,:) = particleWeight(i,:)/sum(particleWeight);
    end
    
    for i = 1:num
        if normalWeight(i,:) > 1/num
            nWeight = normalWeight(1,:)
        end
    end
    
    %% Write code for resampling your particles
    
    cdfWeight = cumsum(normalWeight);
    newParticles(num,1) = BotSim;
    %particles(i) = BotSim(modifiedMap);
    
    randomSample = rand(1, num);
    
    x = 0;
    y = cdfWeight(1);
    for i = 1:num %This loop iterates over each entry of randomSample
        for j = 1:num %This loop iterates over each area (entry) of cdfWeight
            if x <= randomSample(i) < y
                newParticles(i) = particles(j);
                break;
            else
                x = cdfWeight(j);
                y = cdfWeight(j+1);
            end
        end
        x = 0;
        y = cdfWeight(1);
    end
        
    %% Write code to check for convergence   
	
    
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i = 1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        botSim.drawScanConfig(); %draws current scan configuration
        [distances, crossingPoints] = botSim.ultraScan(); %get a scan from the real robot.
        scatter(crossingPoints(:,1),crossingPoints(:,2),'marker','o','lineWidth',3); %draws crossingpoints
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
