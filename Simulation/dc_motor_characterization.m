clc
clear
test1 = readtable("Test1_600PWM.txt");
test2 = readtable("Test2_1200PWM.txt");
test3 = readtable("Test3_1800PWM.txt");
test4 = readtable("Test4_2400PWM.txt");

motorEncoderCntPerRev = 1600;
motorArmCntPerRev = 2400;

motorRpm1 = test1.Var2(5:100);
motorRpm2 = test2.Var2(5:100);
motorRpm3 = test3.Var2(61:156);
motorRpm4 = test4.Var2(5:100);
rpm_samples = 1:96;

motor_rpm_1_ = -(((motorRpm1(96) - motorRpm1(1)) / motorEncoderCntPerRev) / 960) * 1000 * 60;
motor_rpm_2_ = -(((motorRpm2(96) - motorRpm2(1)) / motorEncoderCntPerRev) / 960) * 1000 * 60;
motor_rpm_3_ = -(((motorRpm3(96) - motorRpm3(1)) / motorEncoderCntPerRev) / 960) * 1000 * 60;
motor_rpm_4_ = -(((motorRpm4(96) - motorRpm4(1)) / motorEncoderCntPerRev) / 960) * 1000 * 60;

rpm_a_(1) = 0;
rpm_a_(2) = 0;
rpm_a_(3) = 0;
rpm_a_(4) = motor_rpm_2_;
rpm_a_(5) = motor_rpm_3_;
rpm_a_(6) = motor_rpm_4_;

pwm_a_ = [0, 999, 1000, 2000, 3000, 4000];

delta_RPM_motor = rpm_a_(6) - rpm_a_(3);
delta_PWM_motor = 0.5-0.125;

mPwmRpm = delta_RPM_motor / delta_PWM_motor;

pwm_a_ = pwm_a_ / 8000;

subplot(1,1,1)
plot(pwm_a_, rpm_a_);
xlabel("PWM Duty Cycle");
ylabel("Motor RPM");
title("Motor RPM vs PWM Duty Cycle");
grid on