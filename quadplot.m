grid on;
t_plot = time_steps;

subplot(3, 1, 1);
plot(t_plot,z_plot);
xlabel('Time');
ylabel('Z - position');

subplot(3, 1, 2);
plot(t_plot,vel_plot);
xlabel('Time');
ylabel('Velocity');

subplot(3, 1, 3);
plot(t_plot,acc_plot);
xlabel('Time');
ylabel('Acceleration');

%{
subplot(3, 2, 4);
plot(t_plot,psi_plot);
xlabel('Time');
ylabel('psi- angle');

subplot(3, 2, 5);
plot(t_plot,omega_plot);
xlabel('Time');
ylabel('Angular Velocity');

subplot(3, 2, 6);
plot(t_plot,alpha_plot);
xlabel('Time');
ylabel('Angular Acceleration');
%}