function timeStamp = getTimeStamp()

timeStamp = clock;
timeStamp = sprintf('%.4d%.2d%.2d%.2d%.2d%.2d',timeStamp(1),timeStamp(2),timeStamp(3),timeStamp(4),timeStamp(5),round(timeStamp(6)));
