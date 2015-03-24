load('dataLOG_23-Mar-2015 11-40-25.mat')
dataLOG1 = dataLOG;
load('dataLOG_23-Mar-2015 11-41-13.mat')
dataLOG2 = dataLOG;
load('dataLOG_23-Mar-2015 11-42-15.mat')
dataLOG3 = dataLOG;
load('dataLOG_23-Mar-2015 11-42-40.mat')
dataLOG4 = dataLOG;
load('dataLOG_23-Mar-2015 11-43-09.mat')
dataLOG5 = dataLOG;
load('dataLOG_23-Mar-2015 11-43-51.mat')
dataLOG6 = dataLOG;
load('dataLOG_23-Mar-2015 11-44-17.mat')
dataLOG7 = dataLOG;
load('dataLOG_23-Mar-2015 11-44-54.mat')
dataLOG8 = dataLOG;
load('dataLOG_23-Mar-2015 11-45-22.mat')
dataLOG9 = dataLOG;
load('dataLOG_23-Mar-2015 11-45-47.mat')
dataLOG10 = dataLOG;

plot(dataLOG1.Time, dataLOG1.ErrorZ, dataLOG2.Time, dataLOG2.ErrorZ, ...
     dataLOG2.Time, dataLOG3.ErrorZ, dataLOG2.Time, dataLOG4.ErrorZ, ...
     dataLOG2.Time, dataLOG5.ErrorZ, dataLOG2.Time, dataLOG6.ErrorZ, ...
     dataLOG2.Time, dataLOG7.ErrorZ, dataLOG2.Time, dataLOG8.ErrorZ, ...
     dataLOG2.Time, dataLOG9.ErrorZ, dataLOG2.Time, dataLOG10.ErrorZ);

dataLOGERRORZ = [dataLOG1.ErrorZ; dataLOG2.ErrorZ; dataLOG3.ErrorZ; ...
                 dataLOG4.ErrorZ; dataLOG5.ErrorZ; dataLOG6.ErrorZ; ...
                 dataLOG7.ErrorZ; dataLOG8.ErrorZ; dataLOG9.ErrorZ; ...
                 dataLOG10.ErrorZ];
maxdataLOGERRORZ = max(dataLOGERRORZ);
mindataLOGERRORZ = min(dataLOGERRORZ);
figure;
plot(dataLOG1.Time, detrend(mindataLOGERRORZ), dataLOG1.Time, detrend(maxdataLOGERRORZ));
plot(dataLOG1.Time, mindataLOGERRORZ, dataLOG1.Time, maxdataLOGERRORZ);

figure;
plot(dataLOG1.Time, dataLOG1.toolTRAJ(3,:), dataLOG2.Time, dataLOG2.toolTRAJ(3,:), ...
     dataLOG2.Time, dataLOG3.toolTRAJ(3,:), dataLOG2.Time, dataLOG4.toolTRAJ(3,:), ...
     dataLOG2.Time, dataLOG5.toolTRAJ(3,:), dataLOG2.Time, dataLOG6.toolTRAJ(3,:), ...
     dataLOG2.Time, dataLOG7.toolTRAJ(3,:), dataLOG2.Time, dataLOG8.toolTRAJ(3,:), ...
     dataLOG2.Time, dataLOG9.toolTRAJ(3,:), dataLOG2.Time, dataLOG10.toolTRAJ(3,:));
