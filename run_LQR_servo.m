%PID control for a 2DOF robot arm

cfg.Ts = 0.045;
load referenceTraj2
cfg.ref = q_ref;
cfg.x = [cfg.ref(1,1) cfg.ref(2,1) 0 0]';
cfg.u= [0;0];
cfg.N = 270;
cfg.e = cfg.ref(:,1)-cfg.x(1:2);
res=abs(res);
cfg.PID.Kp1 = res(1);
cfg.PID.Kp2 = res(2);
cfg.PID.Ki1 = res(3);
cfg.PID.Ki2 = res(4);
cfg.PID.Kd1 = res(5);
cfg.PID.Kd2 = res(6);
for i =1:cfg.N
    cfg.e(:,i+1) = cfg.ref(:,i)-cfg.x(1:2,i);
    cfg.u(:,i) = calcPID(cfg.e,cfg.Ts,cfg.PID);
    cfg.x(:,i+1)= MyModel(cfg.x(:,i),cfg.u(:,i));
end

t= 0:cfg.Ts:(cfg.Ts*(cfg.N-1));
plot(t,cfg.ref(1,:), 'r--',t,cfg.x(1,1:end-1), 'b','LineWidth',1.7);
grid;
h_legend=legend('$q_{1ref}$','$q_{1sim}$' ,3);
set(h_legend,'FontSize',20,'Interpreter','latex');
set(gca,'FontSize',20);
xlabel('Time [s]');
ylabel('Angles [rad]')
xlim([0 12.20]);
figure
plot(t,cfg.ref(2,:), 'r--',t,cfg.x(2,1:end-1), 'b','LineWidth',1.7);
grid;
h_legend=legend('$q_{2ref}$','$q_{2sim}$' ,2);
set(h_legend,'FontSize',20,'Interpreter','latex');
set(gca,'FontSize',20);
xlabel('Time [s]');
ylabel('Angles [rad]')
xlim([0 12.20]);
figure
plot(t,q_ref(1,:), 'r--',t,q_ref(2,:),'b','LineWidth',1.7);
grid;
h_legend=legend('$q_{1ref}$','$q_{2ref}$' ,2);
set(h_legend,'FontSize',20,'Interpreter','latex');
set(gca,'FontSize',20);
xlabel('Time [s]');
ylabel('Angles [rad]')
xlim([0 12.20]);
%