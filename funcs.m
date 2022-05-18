close all
clear all

% configuration for simulation
numOfAgents = 100;
arenaSize = 5; % 
meanSpeed = 0.1;
angleInertia = 0.7; % if you walk in a certain direction
degreePowerLaw = 3; % adjust the distribution of power law, determines the variation of friendliness
neighborsSpeedFactor = 0.005; % slow down the speed when encounter neighbor % quite decisive for cluster formation
fieldR = 0.5; % radius of the field of recognition
numOfSteps = 10000; % number of steps for simulation

agents = generateAgents(numOfAgents, arenaSize, meanSpeed, ...
    angleInertia, degreePowerLaw, neighborsSpeedFactor);
xy = [agents.xy];
temp = [agents.angleWeights];
color = [temp.neighbors];
x = xy(1:2:end);
y = xy(2:2:end);
h = scatter(x, y, 80, color, 'filled','MarkerFaceAlpha', 0.6);
colormap(jet)
col = colorbar
ylabel(col, 'friendliness')
xlim([0 arenaSize]);
ylim([0 arenaSize]);
for i1 = 1:numOfSteps
    agents = moveAgnets(agents, arenaSize, fieldR);
    xy = [agents.xy];
    x = xy(1:2:end);
    y = xy(2:2:end);
    set(h, 'Xdata', x, 'YData', y)
    drawnow
%     pause(0.01)
end

% function to generate agents
function [agents] = generateAgents(numOfAgents, arenaSize, meanSpeed, ...
        angleInertia, degreePowerLaw, neighborsSpeedFactor)
    for i1 = 1:numOfAgents
        agents(i1).xy = rand(1, 2) * arenaSize; % assign random xy axis for each agent
        agents(i1).angle = 2*pi*rand; % assign a random angle of movement
        agents(i1).speed = meanSpeed; % assign a fixed speed of movment
        angleNeighbors = rand^degreePowerLaw;%abs(randn);%exprnd(0.5); %rand^degreePowerLaw;
        agents(i1).angleWeights.neighbors = angleNeighbors; % assign friendliness (tendency to move towards neighbors)
        angleInertia2 = angleInertia * (1 - angleNeighbors); % assign inertia (tendency of keep the same direction)
        agents(i1).angleWeights.inertia = angleInertia2;
        angleRandom = 1 - angleInertia2 - angleNeighbors; % assign tendency of change direction by random
        agents(i1).angleWeights.random = angleRandom;
        agents(i1).neighborsSpeedFactor = neighborsSpeedFactor; % initiate neighbour speed factor
        agents(i1).neighbors = [];
        
    end
end

function neighbors = findNeighbors(agentID, distances, fieldR)
    neighbors = find(distances(agentID, :) <= fieldR);
    neighbors(neighbors == agentID) = [];
end

function [agents] = moveAgnets(agents, arenaSize, fieldR)
    runOrder = randperm(length(agents));
    for i1 = 1:length(agents)
        distances = distancesCalc(agents, arenaSize);
        agent = agents(runOrder(i1));
        agent.neighbors = [];
        agent.neighbors = findNeighbors(runOrder(i1), distances, fieldR);
        speed = agent.speed + 0.1 * agent.speed * randn;
        angleWeights = agent.angleWeights;
        x1 = agent.xy(1);
        y1 = agent.xy(2);
        randomAngle = 2*pi*rand;
        if isempty(agent.neighbors) % if no neighbors within view
            inertia = 1 - angleWeights.random;
            agent.angle = circularMean([agent.angle, randomAngle], ...
                [inertia, angleWeights.random]);
        else
            numOfNeighbors = length(agent.neighbors);
            neighborsAngle = nan(1, numOfNeighbors);
            for i2 = 1:numOfNeighbors
                neighborX = agents(agent.neighbors(1, i2)).xy(1);
                neighborY = agents(agent.neighbors(1, i2)).xy(2);
                dx = neighborX - x1;
                dy = neighborY - y1;
                if abs(dx) > 0.5 * arenaSize
                    if dx >= 0
                        neighborX = x1 + arenaSize - dx;
                    else
                        neighborX = - x1 - arenaSize + dx;
                    end
                    dx = neighborX - x1;
                end
                if abs(dy) > 0.5 * arenaSize
                    if dx >= 0
                        neighborY = y1 + arenaSize - dy;
                    else
                        neighborY = - y1 - arenaSize + dy;
                    end
                    dy = neighborY - y1;

                end
                neighborsAngle(i2) = atan2(dy, dx);
            end
            neighborsMeanAngle = circularMean(neighborsAngle, ones(1, numOfNeighbors));
            agent.angle = circularMean([agent.angle, randomAngle, ...
                neighborsMeanAngle], [angleWeights.inertia, ...
                angleWeights.random, angleWeights.neighbors]);
            speed = speed - agent.neighborsSpeedFactor * numOfNeighbors + 0.1 * agent.speed * randn;
            clearvars neighborX neighborY dx dy distX distY
        end
        if speed < 0
            speed = 0;
        end
        x2 = x1 + speed * cos(agent.angle);
        y2 = y1 + speed * sin(agent.angle);
        [x3, y3] = PBC(x2, y2, arenaSize);
        agent.xy = [x3, y3];
        agents(runOrder(i1)) = agent;
        clearvars speed
    end
end


function [x2, y2] = PBC(x1, y1, arenaSize)
    if x1 >= arenaSize
        x2 = x1 - arenaSize;
    elseif x1 < 0
        x2 = arenaSize + x1;
    else
        x2 = x1;
    end
    if y1 >= arenaSize
        y2 = y1 - arenaSize;
    elseif y1 < 0
        y2 = arenaSize + y1;
    else
        y2 = y1;
    end
end

function meanAlpha = circularMean(alpha, weights)
    cosAlpha = weights .* cos(alpha);
    sinAlpha = weights .* sin(alpha);
    meanAlpha = atan2(sum(sinAlpha), sum(cosAlpha));
end

function distances = distancesCalc(agents, arenaSize)
    positions = [agents(:).xy];
    xs = positions(1:2:end);
    xDist = abs(xs' - xs);
    ys = positions(2:2:end);
    yDist = abs(ys' - ys);
    xDist(xDist > 0.5 * arenaSize) = arenaSize - xDist(xDist > 0.5 * arenaSize);
    yDist(yDist > 0.5 * arenaSize) = arenaSize - yDist(yDist > 0.5 * arenaSize);
    distances = sqrt(xDist.^2 + yDist.^2);
end
    

