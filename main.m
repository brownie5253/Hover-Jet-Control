%% Cart Pendulum
clc
clear all
close all

%% Parameters
Aircraft_parameters_10486925();

%% Controller design
point = "A";

%check controllability
test = rank(ctrb(A,B)) == length(A);
assert(test, strcat("Error system not controllable at point",point))

% compute K gains
R = eye(2);
Q = eye(6)*5;
[K,~,CLP] = lqr(A,B,Q,R);

%% observer designs
test = rank(obsv(A,C)) == length(A);
assert(test, strcat("Error system not observable at point",point))

%Observer L gains
pl = CLP*10;
L = place(A', C', pl)';

%% Run Sim

%ode solver settings
h = 0.02;
stoptime = 10;

use_state = 0;
use_output = 1;

%run Outputfeedback sim ( Q = 5, poles = 10x)
Lin_Sim2 = sim('Linear_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');
NL_Sim2 = sim('NL_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');

use_state = 1;
use_output = 0;

%Run state-feedback ( Q = 5, poles = 10x)
Lin_Sim1 = sim('Linear_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');
NL_Sim1 = sim('NL_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');

%Q=1
Q = eye(6);
[K,~,~] = lqr(A,B,Q,R);
Lin_Sim3 = sim('Linear_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');

%Q=10
Q = eye(6)*10;
[K,~,~] = lqr(A,B,Q,R);
Lin_Sim4 = sim('Linear_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');

%Q=15
Q = eye(6)*15;
[K,~,~] = lqr(A,B,Q,R);
Lin_Sim5 = sim('Linear_System_10486925','Solver','ode4','FixedStep','h','StopTime','stoptime');

%% plot results

%combine states and force
Lin_data1 = [Lin_Sim1.x, Lin_Sim1.F];
Lin_hat1 =  Lin_Sim1.x_hat+x_bar';
Lin_data2 = [Lin_Sim2.x, Lin_Sim2.F];
Lin_hat2 = Lin_Sim2.x_hat+x_bar';
NL_data1 = [NL_Sim1.x, NL_Sim1.F];
NL_hat1 =  NL_Sim1.x_hat+x_bar';
NL_data2 = [NL_Sim2.x, Lin_Sim2.F];
NL_hat2 = NL_Sim2.x_hat+x_bar';

%diff Q tests
Lin_data3 = [Lin_Sim3.x, Lin_Sim3.F];
Lin_data4 = [Lin_Sim4.x, Lin_Sim4.F];
Lin_data5 = [Lin_Sim5.x, Lin_Sim5.F];

r2d = [1, 1, 180/pi, 1, 1, 180/pi,1, 1];

labels = ["x1, Pos x [m]", "x2, Pos y [m]", "x3, Pitch [deg]", "x4, Vel x [m/s]",...
    "x5, Vel y [m/s]", "x6, Pitch vel [deg/s]", "U1, Input [N]", "U2, Input [N]"];

figure
sgtitle('Aircraft States through state-feedback control');
for ii = 1:6
    subplot(8,1,ii)
    plot(Lin_Sim1.tout, r2d(ii)*Lin_data1(:,ii),'b-')
    hold on
    plot(NL_Sim1.tout, r2d(ii)*NL_data1(:,ii),'g--')
    hold off
    xlabel('time [s]')
    ylabel(labels{ii})
    legend('Lin State estimations', 'NL State estimations')
end
 subplot(8,1,7);
 plot(Lin_Sim1.tout, Lin_data1(:,7),'r-',Lin_Sim1.tout, NL_data1(:,7),'b--')
 xlabel("Time [s]")
 ylabel(labels{7})
 legend(strcat("linearised Input"),...
            strcat("non-linearised Input"))
 
 subplot(8,1,8);
 plot(Lin_Sim1.tout, Lin_data1(:,8),'r', Lin_Sim1.tout, NL_data1(:,8),'b--')
 xlabel("Time [s]")
 ylabel(labels{8})
 legend(strcat("linearised Input"),...
            strcat("non-linearised Input"))

 figure
 sgtitle('Effect of Q on states');
 for ii = 1:6
     subplot(8,1,ii)
     plot(Lin_Sim1.tout, r2d(ii)*Lin_data3(:,ii),'b',Lin_Sim1.tout, r2d(ii)*Lin_data1(:,ii),'r', Lin_Sim1.tout, r2d(ii)*Lin_data4(:,ii),'g', Lin_Sim1.tout, r2d(ii)*Lin_data5(:,ii),'k')
     xlabel('time [s]')
     ylabel(labels{ii})
     legend('Q=1, Lin State estimations','Q=5, Lin State estimations','Q=10, Lin State estimations', 'Q=15, Lin State estimations')
 end
  subplot(8,1,7);
  plot(Lin_Sim1.tout, Lin_data3(:,7),'b',Lin_Sim1.tout, Lin_data1(:,7),'r', Lin_Sim1.tout, Lin_data4(:,7),'g', Lin_Sim1.tout, Lin_data5(:,7),'k')
  xlabel("Time [s]")
  ylabel(labels{7})
  legend('Q=1, Lin State estimations','Q=5, Lin State estimations','Q=10, Lin State estimations', 'Q=15, Lin State estimations')
  
  subplot(8,1,8);
  plot(Lin_Sim1.tout, Lin_data3(:,8),'b',Lin_Sim1.tout, Lin_data1(:,8),'r', Lin_Sim1.tout, Lin_data4(:,8),'g', Lin_Sim1.tout, Lin_data5(:,8),'k')
  xlabel("Time [s]")
  ylabel(labels{8})
  legend('Q=1, Lin State estimations','Q=5, Lin State estimations','Q=10, Lin State estimations', 'Q=15, Lin State estimations')      

figure
sgtitle(strcat("Comparision of state-feedback VS observer estimates"));
for ii = 1:6
    subplot(6,1,ii);
    plot(Lin_Sim1.tout,r2d(ii)*Lin_data1(:,ii),'b-',Lin_Sim1.tout, r2d(ii)*Lin_hat1(:,ii),'r-')
    hold on
    plot(NL_Sim1.tout,r2d(ii)*NL_data1(:,ii),'g--',NL_Sim1.tout, r2d(ii)*NL_hat1(:,ii),'k--')
    hold off
    xlabel('time [s]')
    ylabel(labels{ii})
    legend(strcat('x_',num2str(ii),"linearised"),...
            strcat('x_',num2str(ii),"hat linearised"),...
            strcat('x_',num2str(ii),"Non-linear"),...
            strcat('x_',num2str(ii),"hat Non-linear"));
end

figure
sgtitle('Comparision of State-feedback vs Output-feedback Control design');
for ii = 1:8
    subplot(8,1,ii)
    plot(Lin_Sim1.tout, r2d(ii)*Lin_data1(:,ii),'b-', Lin_Sim2.tout, r2d(ii)*Lin_data2(:,ii),'r-')
    hold on
    plot(NL_Sim1.tout, r2d(ii)*NL_data1(:,ii),'g--', NL_Sim2.tout, r2d(ii)*NL_data2(:,ii),'k--')
    hold off
    xlabel('time [s]')
    ylabel(labels{ii})
    legend('State-feedback', 'Observer-feedback', 'NL State-feedback', 'NL Observer-feedback')
end

%% animation 
 Aircraft_Animation_10486925(Lin_Sim1.tout, Lin_Sim1.x(:,1), Lin_Sim1.x(:,2), Lin_Sim1.x(:,3), x_bar(1), x_bar(2), x_bar(3))

















