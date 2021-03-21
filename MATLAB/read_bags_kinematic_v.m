clear all;clc;
bag = rosbag('miso_test_energy.bag');
bsel1 = select(bag,'Topic','/irb6640/velocity_desired');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    v_des(:,i)=msgStructs1{i}.Data;
    Tv_des(i)=bsel1.MessageList.Time(i);
end

bsel1 = select(bag,'Topic','/irb6640/velocity');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    v_safe(:,i)=msgStructs1{i}.Data;
    Tv_safe(i)=bsel1.MessageList.Time(i);
end

bsel1 = select(bag,'Topic','/irb6640/joint/velocity');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    v_act(:,i)=msgStructs1{i}.Data;
    Tv_act(i)=bsel1.MessageList.Time(i);
end

bsel1 = select(bag,'Topic','/h_val');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    h(:,i)=msgStructs1{i}.Data(1);
    if length(msgStructs1{i}.Data) > 1
        h_only(:,i)=msgStructs1{i}.Data(2);
    end
    Th(i)=bsel1.MessageList.Time(i);
end


bsel1 = select(bag,'Topic','/irb6640/torques');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    u_des(:,i)=msgStructs1{i}.Data;
    Tu_des(i)=bsel1.MessageList.Time(i);
end

bsel1 = select(bag,'Topic','/irb6640/torques_actual');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    u_safe(:,i)=msgStructs1{i}.Data;
    Tu_safe(i)=bsel1.MessageList.Time(i);
end

bsel1 = select(bag,'Topic','/irb6640/end_effector');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    ee(:,i)=msgStructs1{i}.Data;
    T_ee(i)=bsel1.MessageList.Time(i);
end
startTime = T_ee(1);

i = 1;
while (sum(ee(:,i) == ee(:,1)) == 3)
    i = i+1;
end
ee(:,1:i) = [];
T_ee(:,1:i) = [];

i = 1;
while (sum(ee(:,i) == ee(:,1)) == 3)
    i = i+1;
end
ee(:,1:i) = [];
T_ee(:,1:i) = [];

i = 1;
while (sum(ee(:,i) == ee(:,1)) == 3)
    i = i+1;
end
ee(:,1:i) = [];
T_ee(:,1:i) = [];

stopTime = T_ee(1);

startTimes = [Tv_des(1),Tv_safe(1),Tv_act(1),Th(1),Tu_des(1), Tu_safe(1),startTime];
Tv_des = double(Tv_des - min(startTimes));
Tv_safe =  double(Tv_safe - min(startTimes));
Tv_act =  double(Tv_act - min(startTimes));
Tu_des =  double(Tu_des - min(startTimes));
Tu_safe =  double(Tu_safe - min(startTimes));
Th =  double(Th - min(startTimes));
stopTime = double(stopTime - min(startTimes))

bag = rosbag('miso_test_no_energy.bag');

bsel1 = select(bag,'Topic','/h_val');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
for i=1:length(msgStructs1)
    h_nf(:,i)=msgStructs1{i}.Data;
    Th_nf(i)=bsel1.MessageList.Time(i);
end

Th_nf =  double(Th_nf - Th_nf(1));
%%
stopTime = stopTime - 0.01;
close all;
figure(1)
if (exist("h_only"))
    sgtitle(strcat('$\alpha_e = 1500$,',' $h_{\textrm{min}} = $', sprintf(' %0.3g',min(h_only)/250), sprintf(', time = %0.4g', stopTime+0.01)),'Interpreter','latex','FontSize',20)
else
    sgtitle(strcat('$\alpha = 1$,',' $h_{\textrm{min}} = $', sprintf(' %0.3g',min(h)/250), sprintf(', time = %0.4g', stopTime+0.01)),'Interpreter','latex','FontSize',20)
end
subplot(4,2,1)
plot(Tv_des,v_des(1,:),'LineWidth',3)
hold on
plot(Tv_safe,v_safe(1,:),'LineWidth',3)
hold on
plot(Tv_act,v_act(1,:),'LineWidth',3)
xlim([0,stopTime])
legend('Desired','Safe','Actual','Interpreter','latex','FontSize',20,'Location','W')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$\dot{q}(1)$ (rad/s)','Interpreter','latex','FontSize',20)

subplot(4,2,3)
plot(Tv_des,v_des(2,:),'LineWidth',3)
hold on
plot(Tv_safe,v_safe(2,:),'LineWidth',3)
hold on
plot(Tv_act,v_act(2,:),'LineWidth',3)
xlim([0,stopTime])
% legend('Desired','Safe','Actual','Interpreter','latex','FontSize',20,'Location','eastoutside')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$\dot{q}(2)$ (rad/s)','Interpreter','latex','FontSize',20)

subplot(4,2,5)
plot(Tv_des,v_des(3,:),'LineWidth',3)
hold on
plot(Tv_safe,v_safe(3,:),'LineWidth',3)
hold on
plot(Tv_act,v_act(3,:),'LineWidth',3)
xlim([0,stopTime])
% legend('Desired','Safe','Actual','Interpreter','latex','FontSize',20,'Location','N')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$\dot{q}(3)$ (rad/s)','Interpreter','latex','FontSize',20)

subplot(4,2,2)
plot(Tu_des,u_des(1,:),'LineWidth',3)
hold on
plot(Tu_safe,u_safe(1,:),'LineWidth',3)
xlim([0,stopTime])
legend('Desired','Safe','Interpreter','latex','FontSize',20,'Location','NW')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$u(1)$ (Nm)','Interpreter','latex','FontSize',20)

subplot(4,2,4)
plot(Tu_des,u_des(2,:),'LineWidth',3)
hold on
plot(Tu_safe,u_safe(2,:),'LineWidth',3)
xlim([0,stopTime])
% legend('Desired','Safe','Interpreter','latex','FontSize',20,'Location','SW')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$u(2)$ (Nm)','Interpreter','latex','FontSize',20)

subplot(4,2,6)
plot(Tu_des,u_des(3,:),'LineWidth',3)
hold on
plot(Tu_safe,u_safe(3,:),'LineWidth',3)
xlim([0,stopTime])
% legend('Desired','Safe','Interpreter','latex','FontSize',20,'Location','NW')
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$u(3)$ (Nm)','Interpreter','latex','FontSize',20)

subplot(4,2,[7,8])
if size(h,1) > 1
plot(Th(10:end),min(h(:,10:end)),'LineWidth',3)
else
  plot(Th(10:end),h(10:end),'LineWidth',3)  
  hold on
  if (exist("h_only"))
  plot(Th(10:end),h_only(10:end),'LineWidth',3) 
  end
end
xlim([0,stopTime])
legend('$h_D$','$\alpha_e h$','Interpreter','latex','FontSize',20)
xlabel('t (s)','Interpreter','latex','FontSize',20)
ylabel('$h(q)$','Interpreter','latex','FontSize',20)
% hold on
% plot(Th_nf,min(h_nf))
