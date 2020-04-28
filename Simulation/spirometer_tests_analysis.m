clc
clear
test1 = readtable("SpirometerTest_Lung.csv");

sample_time_ms = 100;
sample_time_s = 0.1;
max_seconds = 223;

voltage_sampled = transpose(0:(max_seconds * (1/sample_time_s)));
disp_volume_sampled = transpose(0:(max_seconds * (1/sample_time_s)));
delta_disp_volume_sampled = transpose(0:(max_seconds * (1/sample_time_s)));
time_ms = transpose(0:sample_time_s:max_seconds);
samples = 0:(max_seconds * (1/sample_time_s));

current_voltage_row = 1;
current_time_row = 2;
current_flow_row = 2;

test1.VOL_FLOW = -test1.VOL_FLOW;

voltage_sampled(1) = test1.VOLTAGE(current_voltage_row);
disp_volume_sampled(1) = test1.VOL_FLOW(current_flow_row) * sample_time_ms;
delta_disp_volume_sampled(1) = 0;

for i = 2:(max_seconds * (1/sample_time_s) + 1)
    
    if (time_ms(i) > test1.TIMESTAMP(current_time_row))
        current_voltage_row = current_voltage_row + 1;
        current_time_row = current_time_row + 1;
        current_flow_row = current_flow_row + 1;
    end
    
    voltage_sampled(i) = test1.VOLTAGE(current_voltage_row);
    disp_volume_sampled(i) = disp_volume_sampled(i-1) + ...
                            (test1.VOL_FLOW(current_flow_row) * sample_time_ms);
    delta_disp_volume_sampled(i) = disp_volume_sampled(i) - disp_volume_sampled(i-1);
    
end

plot(time_ms, voltage_sampled, ...
    time_ms, delta_disp_volume_sampled);
