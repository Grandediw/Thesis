close all
clear
clc
 %%
% Replace 'COMX' with the actual COM port of your Arduino
arduino = serial('/dev/cu.usbserial-1110','BaudRate',  115200);

fopen(arduino);

num_samples = 600; % Adjust to the number of data points you expect
data = zeros(num_samples, 6); % Columns: Time, PWM Signal, Frequency, Current

disp('Starting data acquisition...');

for i = 1:num_samples
    data_str = fread(arduino, arduino.BytesAvailable, 'char');
    data_str = char(data_str'); % Convert from uint8 to char
    data_str = strtrim(data_str); % Remove leading/trailing whitespace
    
    if isempty(data_str)
        disp('Timeout reached. Exiting loop.');
        break;
    end
    
    data_parts = str2double(strsplit(data_str, ','));
    data(i, :) = data_parts;
end

disp('Data acquisition complete.');

fclose(arduino);
delete(arduino);
clear arduino;

disp('Serial port closed.');

figure;

subplot(3, 1, 1);
plot(data(:, 1), data(:, 2));
xlabel('Time (s)');
ylabel('PWM Signal');
title('Arduino Data Analysis');

subplot(3, 1, 2);
plot(data(:, 1), data(:, 3));
xlabel('Time (s)');
ylabel('Frequency (Hz)');

subplot(3, 1, 3);
plot(data(:, 1), data(:, 4));
xlabel('Time (s)');
ylabel('Current (mA)');
