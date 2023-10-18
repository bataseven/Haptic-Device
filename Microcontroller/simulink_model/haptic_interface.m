
controlLoopFreq = 500; % Hz
countToMillimeter = 29.5 / 99510.0 % 26.1 / 107493;


K = 1; % Controller Gain
m = 15; % Admittance controller mass
b = 45; % Admittance controller damping
k = 1; % Controller spring
dt = 1/ controlLoopFreq; % Time step

object_k = 5; % [N / mm]
object_b = 2.5; %[N*s / mm]

object_width = 20; %[mm]

R = 0 : 1 : 6000;
y = 153.18 * R .^-0.699;

loglog(R, y);
ylim([0 30])
