clear all

%% Initialize
terrainName = 'slope';
widthInXDirection = 1; % in m
widthInYDirection = 1; % in m
inclination = 0.15 / widthInXDirection;
resolution = 1000; % in cells per m

nCellsInXDirection = resolution * widthInXDirection;
nCellsInYDirection = resolution * widthInYDirection;
nCells = nCellsInXDirection * nCellsInYDirection;

terrainMatrix = zeros(nCells, 4 );

%% Generate
for xIndex = 1:nCellsInXDirection
    for yIndex = 1:nCellsInYDirection
        
        cellIndex = (xIndex-1) * nCellsInXDirection + yIndex;
        terrainMatrix(cellIndex, 1) = xIndex;
        terrainMatrix(cellIndex, 2) = yIndex;
        terrainMatrix(cellIndex, 3) = (xIndex-1) * inclination;
        terrainMatrix(cellIndex, 4) = 0;
        
    end
end

%% Plot
index = 1:length(terrainMatrix);
plot3(terrainMatrix(index,1), terrainMatrix(index,2), terrainMatrix(index,3));

%% Save
fileName = [terrainName '.asc'];
save(fileName, 'terrainMatrix', '-ascii', '-double', '-tabs');