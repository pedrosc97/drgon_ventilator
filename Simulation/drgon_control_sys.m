clc
clear

% Inputs
respiratory_frequency = 40;
inhalation = 1;
exhalation = 4;
set_point_ml = 800;

% System Model Constants
ARM_ANGLE_OFFSET_DEG = 10;
PWM_MAX_PULSE_NUMBER = 8000;
MOTOR_MAX_VOLTAGE = 12;

K_P = 650;
K_I = 25;
K_D = 3.7;

% Helper Constants
SECONDS_PER_MINUTE = 60;
MS_PER_SECOND = 1000;
RPM_TO_RAD_S = 0.104719755119660;
RAD_TO_DEG = 180 / 3.14159;

% Simulation Settings
simu_time_ms = 40000;
sample_time_ms = 10;
sample_time_s = sample_time_ms / MS_PER_SECOND;
samples_array = transpose(0:sample_time_ms:(simu_time_ms - 1));

%%%%% Algorithm %%%%%%%

% Volume to Angle Conversion
set_point_deg = (-0.0000421 * (set_point_ml ^ 2)) + (0.0545 * set_point_ml) + 3.984;
peak_angle_rad = (set_point_deg * 3.14159 / 180);

% Time Calculation
respiration_cycle_time = SECONDS_PER_MINUTE / respiratory_frequency;
inhalation_time = respiration_cycle_time * (inhalation / (inhalation + exhalation));
exhalation_time = respiration_cycle_time * (exhalation / (inhalation + exhalation));
sample_number = respiration_cycle_time * MS_PER_SECOND / sample_time_ms;

% Cycle Times
t_inhalation_trajectory_begin = 0;
t_inhalation_trajectory_end = inhalation_time;
t_exhalation_trajectory_begin = t_inhalation_trajectory_end;
t_exhalation_trajectory_end = t_inhalation_trajectory_end + exhalation_time;

% Trajectory Generation
a_inhalation = GenerateTrajectoryCoef(t_inhalation_trajectory_begin, ...
                                      t_inhalation_trajectory_end, ...
                                      0, ...
                                      peak_angle_rad);
                                  
a_exhalation = GenerateTrajectoryCoef(t_exhalation_trajectory_begin, ...
                                      t_exhalation_trajectory_end, ...
                                      peak_angle_rad, ...
                                      0);

% Store cycle trajectory angle and angular velocity in an array
theta_d = zeros([sample_number, 1]);
omega_d = zeros([sample_number, 1]);

inhalation_samples_begin = 1;
inhalation_samples_end = cast(t_inhalation_trajectory_end * MS_PER_SECOND / sample_time_ms, "uint16");
exhalation_samples_begin = inhalation_samples_end + 1;
exhalation_samples_end = cast(t_exhalation_trajectory_end * MS_PER_SECOND / sample_time_ms, "uint16");

for j = inhalation_samples_begin:inhalation_samples_end
    theta_d(j) = a_inhalation(1) + a_inhalation(2) * (samples_array(j) / MS_PER_SECOND) + ...
                   a_inhalation(3) * (samples_array(j) / MS_PER_SECOND) ^ 2 + ...
                   a_inhalation(4) * (samples_array(j) / MS_PER_SECOND) ^ 3 + ... 
                   a_inhalation(5) * (samples_array(j) / MS_PER_SECOND) ^ 4 + ...
                   a_inhalation(6) * (samples_array(j) / MS_PER_SECOND) ^ 5;
    omega_d(j) = a_inhalation(2) + 2 * a_inhalation(3) * (samples_array(j) / MS_PER_SECOND) + ...
                   3 * a_inhalation(4) * (samples_array(j) / MS_PER_SECOND) ^ 2 + ... 
                   4 * a_inhalation(5) * (samples_array(j) / MS_PER_SECOND) ^ 3 + ...
                   5 * a_inhalation(6) * (samples_array(j) / MS_PER_SECOND) ^ 4;
end

for j = exhalation_samples_begin:exhalation_samples_end
    theta_d(j) = a_exhalation(1) + a_exhalation(2) * (samples_array(j) / MS_PER_SECOND) + ...
                   a_exhalation(3) * (samples_array(j) / MS_PER_SECOND) ^ 2 + ...
                   a_exhalation(4) * (samples_array(j) / MS_PER_SECOND) ^ 3 + ... 
                   a_exhalation(5) * (samples_array(j) / MS_PER_SECOND) ^ 4 + ...
                   a_exhalation(6) * (samples_array(j) / MS_PER_SECOND) ^ 5;
    omega_d(j) = a_exhalation(2) + 2 * a_exhalation(3) * (samples_array(j) / MS_PER_SECOND) + ...
                   3 * a_exhalation(4) * (samples_array(j) / MS_PER_SECOND) ^ 2 + ... 
                   4 * a_exhalation(5) * (samples_array(j) / MS_PER_SECOND) ^ 3 + ...
                   5 * a_exhalation(6) * (samples_array(j) / MS_PER_SECOND) ^ 4;
end

% PID Control
error = zeros([sample_number, 1]);
voltage = zeros([sample_number, 1]);
theta = zeros([sample_number, 1]);
omega = zeros([sample_number, 1]);

% PID
omega(1) = 0;
theta(1) = 0;
error(1) = theta_d(1) - theta(1);
error_integration = ((error(1)) / 2 * sample_time_s);
voltage(1) = (K_P * error(1)) + (K_I * error_integration) + (K_D * (omega_d(1) - omega(1)));
if (voltage(1) > 12)
        voltage(1) = 12;
elseif (voltage(1) < -12)
        voltage(1) = -12;
end
motor_pwm_duty_cycle = voltage(1) / MOTOR_MAX_VOLTAGE;
% For application, duty cycle must be mapped due to rpm being 0 below
% 0.125 in real model.
% End PID

for i = 2:exhalation_samples_end
    
    % Plant Model
    arm_rpm = SimulatePlant(motor_pwm_duty_cycle, theta(i-1));
    % End Plant Model
    
    % PID
    omega(i) = arm_rpm * RPM_TO_RAD_S;
    theta(i) = ((omega(i) + omega(i-1)) / 2 * sample_time_s) + theta(i-1);
    error(i) = theta_d(i) - theta(i);
    error_integration = ((error(i) + error(i-1)) / 2 * sample_time_s) + error(i-1);
    voltage(i) = (K_P * error(i)) + (K_I * error_integration) + (K_D * (omega_d(i) - omega(i)));

    if (voltage(i) > 12)
        voltage(i) = 12;
    elseif (voltage(i) < -12)
        voltage(i) = -12;
    end 
    motor_pwm_duty_cycle = voltage(i) / MOTOR_MAX_VOLTAGE;
    % PID
end

% Plot Results
subplot(2,1,1)
plot(samples_array(inhalation_samples_begin:exhalation_samples_end), ...
     theta_d(inhalation_samples_begin:exhalation_samples_end), ...
     samples_array(inhalation_samples_begin:exhalation_samples_end), ...
     theta(inhalation_samples_begin:exhalation_samples_end));
legend("THETA_D", "THETA", 'Location', 'northeast');
xlabel("Time [ms]");
ylabel("Angle [rad]");
title_string = sprintf("Arm Angular Position (Vol = %d mL, Freq = %d cpm, I:E = %d:%d)", ...
                        set_point_ml, respiratory_frequency, inhalation, exhalation);
title(title_string);
grid on

subplot(2,1,2)
plot(samples_array(inhalation_samples_begin:exhalation_samples_end), ...
     omega_d(inhalation_samples_begin:exhalation_samples_end), ...
     samples_array(inhalation_samples_begin:exhalation_samples_end), ...
     omega(inhalation_samples_begin:exhalation_samples_end));
legend("OMEGA_D", "OMEGA", 'Location', 'northeast');
xlabel("Time [ms]");
ylabel("Angular Velocity [rad/s]");
title_string = sprintf("Arm Angular Velocity (Vol = %d mL, Freq = %d cpm, I:E = %d:%d)", ...
                        set_point_ml, respiratory_frequency, inhalation, exhalation);
title(title_string);
grid on

% Helper Functions
function a = GenerateTrajectoryCoef(t_zero, t_end, angle_zero, angle_end)
    v_zero = 0;
    v_end = 0;
    alpha_zero = 0;
    alpha_end = 0;

    M = [ 1, t_zero, t_zero^2, t_zero^3,   t_zero^4,    t_zero^5;
          0, 1,      2*t_zero, 3*t_zero^2, 4*t_zero^3,  5*t_zero^4;
          0, 0,      2,        6*t_zero,   12*t_zero^2, 20*t_zero^3;
          1, t_end,  t_end^2,  t_end^3,    t_end^4,     t_end^5;
          0, 1,      2*t_end,  3*t_end^2,  4*t_end^3,   5*t_end^4;
          0, 0,      2,        6*t_end,    12*t_end^2,  20*t_end^3;]; 

    b = [angle_zero, v_zero, alpha_zero, angle_end, v_end, alpha_end];
    a = M \ transpose(b);
end

% Plant Model
function arm_rpm = SimulatePlant(duty_cycle, theta)
   
    ARM_ANGLE_OFFSET_DEG = 10;
    RAD_TO_DEG = 180 / 3.14159; 
    motor_rpm = 113.33 * duty_cycle;
    current_gamma = (theta * RAD_TO_DEG) - ARM_ANGLE_OFFSET_DEG;
    rpm_arm_ratio = -0.000043516 * current_gamma ^ 2 - 0.00065146 *  current_gamma + 0.24067;
    arm_rpm = motor_rpm * rpm_arm_ratio;
    
end


