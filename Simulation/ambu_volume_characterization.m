angles = 5:0.1:21;
eq_2400 = 5:0.1:21;
eq_3000 = 5:0.1:21;
eq_3600 = 5:0.1:21;
eq_4200 = 5:0.1:21;
eq_4800 = 5:0.1:21;

eq_2400_coef = [1.2326, 0.2652, -16.815, 0.9958];
eq_3000_coef = [1.0596, 7.8901, -41.375, 0.997];
eq_3600_coef = [1.0487, 10.428, -46.162, 0.9975];
eq_4200_coef = [0.9305, 14.557, -65.484, 0.9961];
eq_4800_coef = [1.0461, 12.327, -47.169, 0.9966];

for i = 1:161
    eq_2400(i) = eq_2400_coef(1) * (angles(i) ^ 2) + ...
                 eq_2400_coef(2) * angles(i) + ...
                 eq_2400_coef(3);
             
    eq_3000(i) = eq_3000_coef(1) * (angles(i) ^ 2) + ...
                 eq_3000_coef(2) * angles(i) + ...
                 eq_3000_coef(3);
    
    eq_3600(i) = eq_3600_coef(1) * (angles(i) ^ 2) + ...
                 eq_3600_coef(2) * angles(i) + ...
                 eq_3600_coef(3);
                 
    eq_4200(i) = eq_4200_coef(1) * (angles(i) ^ 2) + ...
                 eq_4200_coef(2) * angles(i) + ...
                 eq_4200_coef(3);
    
    eq_4800(i) = eq_4800_coef(1) * (angles(i) ^ 2) + ...
                 eq_4800_coef(2) * angles(i) + ...
                 eq_4800_coef(3);
end

plot(angles, eq_2400, angles, eq_3000, angles, eq_3600, angles, eq_4200, angles, eq_4800);
xlabel("Angle Step Input [°]");
ylabel("Displaced Volume [mL]");
legend("D = 0.5 (R = 0.9958)", "D = 0.625 (R = 0.997)", ...
       "D = 0.75 (R = 9975)", "D = 0.875 (R = 0.9961)", ...
       "D = 1.0 (R = 0.9966)", "Location", "northwest");
title("Displaced Volume curves for different Duty Cycles");
grid on