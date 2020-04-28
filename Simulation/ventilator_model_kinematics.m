diameter = 0.060;
r = diameter / 2;

d1 = 0.029;
d2 = 0.1175;
d3 = 0.125 - r;
alpha = 45;

gamma = -10:2:22;
m = 1:17;
rpm = 0:100;
rpm_arm = zeros(101, 17);

for j = 1:17
    for i = 1:101
        theta = alpha - gamma(j);
        omega = rpm(i) * (3.14159 / 30);
        v = omega * r;

        L = (d2^2 + d3^2 - 2 * d2 * d3 * cosd(theta))^(1/2);
        phi = acosd(d1 / L);
        vt = v * sind(phi);
        omega_arm = vt / d2;
        rpm_arm(i, j) = omega_arm * (30 / 3.14159);
    end
    
    m(j) = (rpm_arm(101, j) - rpm_arm(1, j)) / 100;
end

modelFun = @(b,x) b(1).*x.^2 + b(2).*x + b(3);
start = [0.1; 0.1; 22];
nlm = fitnlm(transpose(gamma),transpose(m),modelFun,start)
xx = linspace(-10,25)';

subplot(2,1,1);
plot(rpm, rpm_arm(:, 1), ...
     rpm, rpm_arm(:, 3), ...
     rpm, rpm_arm(:, 5), ...
     rpm, rpm_arm(:, 7), ...
     rpm, rpm_arm(:, 9), ...
     rpm, rpm_arm(:, 11), ...
     rpm, rpm_arm(:, 13), ...
     rpm, rpm_arm(:, 15), ...
     rpm, rpm_arm(:, 17));
xlabel("DC Motor RPM");
ylabel("Arm RPM");
str = sprintf("RPM Relationship between Motor and Arm, D = %dmm", diameter * 1000);
title(str);
grid on

subplot(2,1,2);
plot(gamma, m);
xlabel("Arm Angle [°]");
ylabel("Motor-Arm RPM Ratio");
str = sprintf("Relationship between Arm Angle and Motor-Arm RPM Ratio, D = %dmm", diameter * 1000);
title(str);
grid on

