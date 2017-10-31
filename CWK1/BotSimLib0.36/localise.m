function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%%% Assuming all maps have limsMin = [0 0] -- Could possibly change
limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 5; %sampling resouloution in cm
iterators = dims/res;
iterators = ceil(iterators) + [1 1]; %to counteract 1 based indexing
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

%sets number of scan lines
numScanLines = 20;

%generate some random particles inside the map
num = 300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).setScanConfig(particles(i).generateScanConfig(numScanLines));
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).drawBot(3);
end

botSim.drawMap();
%% Localisation code
maxNumOfIterations = 100;
n = 0;
converged = 0; %The filter has not converged yet
currentConverge = inf;
botSim.setScanConfig(botSim.generateScanConfig(numScanLines));
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan(1,:) = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    
    %particleScans = cell(num, 1); %Creates a cell matrix to contain each particle's scans
    particleScans = zeros(num, numScanLines);
    for i = 1:num
        particleScans(i,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles    
    
    particleDiff = zeros(num,numScanLines);
    averageParticleDiff = zeros(num,1);
    for i = 1:num
        particleDiff(i,:) = abs(botScan - particleScans(i,:)); %Calculates the absolute difference between particle and bot scans
        averageParticleDiff(i) = sum(particleDiff(i,:))/numScanLines; %Average difference between scans of bot and particle
    end
    
    averageParticleDiff;
    
    particleWeight = zeros(num, 1);
    sd = 6;
    damp = 0.01;
    
    particleWeight = normpdf(averageParticleDiff, 0, sd) + damp;
    
    %{
    for i = 1:num
        particleWeight(i) = (exp(-((averageParticleDiff(i)^2)/(sd^2)))) + damp;
    end
    %}
    
    particleWeight;
        
    normalWeight = zeros(num, 1);
    for i = 1:num
        normalWeight(i) = particleWeight(i)/sum(particleWeight);
    end
    
    normalWeight;
    
    %{
    for i = 1:num
        if normalWeight(i) > 1/num
            nWeight = normalWeight(1)
        end
    end
    %}
    
    %% Write code for resampling your particles
    
    cdfWeight = cumsum(normalWeight);
    %newParticles(num,1) = BotSim;
    %particles(i) = BotSim(modifiedMap);
    
    randomSample = rand(1, num);
    
    for i = 1:num %This loop iterates over each entry of randomSample
        x = 0;
        y = cdfWeight(1);
        for j = 1:num %This loop iterates over each area (entry) of cdfWeight
            if x <= randomSample(i) && randomSample(i) < y
                %newParticles(i) = particles(j);
                %if currentConverge >= 200
                    newPos(i,:) = particles(j).getBotPos() + 0.4*(randn(1,2));
                    newAng(i) = particles(j).getBotAng() + 0.1*randn();
                %elseif currentConverge <= 200
                %    newPos(i,:) = particles(j).getBotPos() + 0.1*(randn(1,2));
                %    newAng(i) = particles(j).getBotAng() + 0.1*randn();
                %end
                break;
            else
                x = cdfWeight(j);
                y = cdfWeight(j+1);
            end
        end
    end
    
    for i = 1:num
        particles(i).setBotPos(newPos(i,:));
        particles(i).setBotAng(newAng(i));
    end
    
    %% Write code to check for convergence   
	
    particlePos = zeros(num,2);
    particleAng = zeros(num,2);
    
    for i = 1:num
        particlePos(i,:) = particles(i).getBotPos();
        particleAng(i) = particles(i).getBotAng();
    end
    
    averagePos = (sum(particlePos)/num);
    averageAng = (sum(particleAng)/num);
    
    posDiff = zeros(num,2);
    euclidDist = zeros(num,1);
    
    for i = 1:num
        posDiff(i,:) = averagePos - particlePos(i,:);
        euclidDist(i) = (posDiff(i,1))^2 + (posDiff(i,2))^2;
    end
    
    currentConverge = sum(euclidDist)/num;
    
    convergeConst = 70;
    
    if currentConverge < convergeConst
        a = 'Yay, convergance'
        converged = 1;
        n
        averagePos
        botPos = botSim.getBotPos
    end
    
    if currentConverge < 140
        sd = 2
    elseif currentConverge < 200
        sd = 3
    elseif currentConverge < 400
        sd = 5
    end    
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    for i = 1:(num*0.05)
        particles(i).randomPose(5);
    end
    
    %% Write code to decide how to move next
    
    % A* algorithm
    if converged == 1
        %closedSet = [];
        %openSet = [averagePos];
        %hValues = mapMatrix;
        %gridNode = cell(size(mapMatrix));
    end
    
    if converged == 0
        [maxDist, direc] = max(botScan); %returns longest distance scan line and index in array
        turn = ((direc-1)/numScanLines)*(2*pi);
        move = 0.1*maxDist;
        botSim.turn(turn);
        botSim.move(move);
        for i = 1:num
            particles(i).turn(turn);
            particles(i).move(move);
            if particles(i).insideMap == 0
                particles(i).randomPose(5);
            end
        end
    end
    
    % here they just turn in circles as an example
    turn = pi/4;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i = 1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
        if particles(i).insideMap == 0
            particles(i).randomPose(5);
        end
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawScanConfig(); %draws current scan configuration
        botSim.drawBot(30,'r'); %draw robot with line length 30 and green
        %[distances, crossingPoints] = botSim.ultraScan(); %get a scan from the real robot.
        %scatter(crossingPoints(:,1),crossingPoints(:,2),'marker','o','lineWidth',3); %draws crossingpoints
        particlePos = zeros(num,2);
        for i = 1:num
            particlePos(i,:) = particles(i).getBotPos();
        end
        scatter(particlePos(:,1),particlePos(:,2),'bo');
        %for i =1:num   
        %    particles(i).drawBot(3); %draw particle with line length 3 and default color
        %end
        drawnow;
    end
end

function [gridNodes, goalNode] = initialiseNodes(goal)
    limsMin = min(map); % minimum limits of the map
    limsMax = max(map); % maximum limits of the map
    dims = limsMax-limsMin; %dimension of the map
    res = 5; %sampling resouloution in cm
    iterators = dims/res;
    iterators = ceil(iterators) + [1 1]; %to counteract 1 based indexing
    gridPoints = iterators + [1,1];
    mapMatrix = zeros(gridPoints);
    gridNodes = cell(size(mapMatrix));
    for i = 1:gridPoints(2)
        for j = 1:gridPoints(1)
            pos = limsMin(1) + [j-1 i-1]*res;
            gValue = inf;
            hValue = sqrt((pos(1) - goal(1))^2 + (pos(2) - goal(2))^2);
            xParent = 0;
            yParent = 0;
            gridNodes(i,j) = [pos(1);pos(2);gValue;hValue;xParent;yParent];
        end
    end
end
end
