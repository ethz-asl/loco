clear all

%% Initialize
terrainName = 'wedgeTunnel';
widthInXDirection = 1; % in m
widthInYDirection = 1; % in m
inclination = 0.25 / (widthInYDirection/2);
resolution = 1000; % in cells per m

nCellsInXDirection = resolution * widthInXDirection + 1;
nCellsInYDirection = resolution * widthInYDirection + 1;
nCells = nCellsInXDirection * nCellsInYDirection;

terrainMatrix = zeros(nCells, 4 );

%% Generate
for xIndex = 1:nCellsInXDirection
    for yIndex = 1:nCellsInYDirection
        
        xPosition = xIndex - 1;
        yPosition = yIndex - 1;
        cellIndex = (xIndex-1) * nCellsInXDirection + yIndex;
        
        terrainMatrix(cellIndex, 1) = xPosition;
        terrainMatrix(cellIndex, 2) = yPosition;
        terrainMatrix(cellIndex, 3) = abs(yPosition - resolution * widthInYDirection/2) * inclination;
        terrainMatrix(cellIndex, 4) = 0;

    end
end

%% Plot
index = 1:length(terrainMatrix);
plot3(terrainMatrix(index,1), terrainMatrix(index,2), terrainMatrix(index,3));

%% Save
fileName = [terrainName '.asc'];
save(fileName, 'terrainMatrix', '-ascii', '-double', '-tabs');