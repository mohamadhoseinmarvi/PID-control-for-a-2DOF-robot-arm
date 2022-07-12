%PID control for a 2DOF robot arm
time = [];                      
data = [0;0];                       
N = 270;                         
Ts = 0.045;
err = [0;0];
utstart('5');

t = 0:Ts:((N-1)*Ts);
ref = [0.6*sin(t)
       0.5*sin(t)];
dref = [0.6*cos(t)
        0.5*cos(t)];
ddref = -ref;
time(1) = 0;

Kx = [36 0 13 0
      0 36 0 13];
Kz = [0.6 0.6];
data=[0;0];
zk = [-data(1)+ref(1,1); -data(2)+ref(2,1)]*Ts;
disp('Session started');
tic 
for k = 1:N,

    data(:,k) = utread;

    err(:,k) = q_ref(:,k)-data(:,k);

    if(k == 1)
        dq = [0 0]';
    else
        dq = (data(:,k)-data(:,k-1))/Ts;
    end

    DATA(:,k) = [data(:,k);dq];

    zk(:,k+1) = zk(:,k) + Ts*[ref(1,k)-DATA(1,k); ref(2,k)-DATA(2,k)];

    u(:,k) = -Kx*(-DATA(:,k)+[ref(:,k);dref(:,k)]) + Kz*zk(:,k);
    tau(:,k) = calcFeedCom(DATA(:,k),ddref(:,k),u(:,k));
    utwrite(tau(:,k));

    time(k+1) = toc;
    while(time(k+1)<(time(k)+Ts))
        time(k+1) = toc;
    end  
end
utwrite([0;0]);

utstop;

q_ref= ref
plot_res