tableSerial = serial('COM3', 'BaudRate', 9600, 'DataBits', 8, 'Parity', 'none', 'StopBits', 1, 'FlowControl', 'none');
fopen(tableSerial);
%fprintf(tableSerial,'500R');
fprintf(tableSerial,'0G');
%tableSerial.BytesAvailable
%ID = fgets(tableSerial)
fclose(tableSerial);
%delete tableSerial;
clear tableSerial;