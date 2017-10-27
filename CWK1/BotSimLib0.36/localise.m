function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);


%%% Assuming all maps have limsMin = [0 0] -- Could possibly change
limsMax = max(map); % maximum limits of the map

mapMatrix = zeros(limsMax + [1,1]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations
    if particles(i).
    particles(i).drawBot(3);
end

botSim.drawMap();
%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged = 0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    [distances, crossingPoints] = botSim.ultraScan(); %get a scan from the real robot.
    
    distances
    crossingPoints
    
    %% Write code for updating your particles scans
    
    for i = 1:num
        particleScans(i,:,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles    
    
    
    
    %% Write code for resampling your particles
    
    %if particles(i)
    
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 0;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
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
        scatter(crossingPoints(:,1),crossingPoints(:,2),'marker','o','lineWidth',3); %draws crossingpoints
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
