clc; clear all;
%% Input - Initial State of Vehicles
n_f=input('Enter Number of moving vehicles or followers: ');
xyO=zeros(n_f,4);
xyO_1=zeros(n_f,4);
disp('Enter initial global posture info (x, y, theta, omega) for the N vehicles and sensors');
for i=1:n_f 
    for j=1:4
        if j==1
            w1=sprintf('Enter x (for vehicle %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for vehicle %d) : ',i);
        end
        if j==3
            w1=sprintf('Enter theta [in degree](for vehicle %d) : ',i);
        end
        if j==4
            w1=sprintf('Enter omega [in radians](for sensor %d) : ',i);
        end
        xyO(i,j)=input(w1);
        xyO_1(i,j)=xyO(i,j);
    end
end
xyO_1(:,3)=deg2rad(xyO_1(:,3));
%% Consensus Analysis
clc;
xyO=xyO_1;
f_x=zeros(1000,n_f);
f_y=zeros(1000,n_f);
k = 0.013;
e = 0.25;
u0 = k;
w0 = 1;%6*pi*k/e;
u = zeros(n_f,2);
p_val = 1;

for time=1:1:1000
    f_x(p_val,:)=xyO(:,1);
    f_y(p_val,:)=xyO(:,2);
    p_val=p_val+1;
    
    for i=1:n_f
        r1 = zeros(n_f,1);
        for j=1:n_f
            if j~=i
                if norm(xyO(i,1:2)-xyO(j,1:2)) > (3*pi*u0)/w0
                    r1(i) = r1(i)+ norm(xyO(i,1:2)-xyO(j,1:2));
                    u(i,:) = u(i,:)+(xyO(j,1:2)-xyO(i,1:2));
                end
            end
        end
        u(i,:)=u(i,:)*(k/(1+r1(i)));    
    end
    
    tspan=[0 1];
    for i=1:n_f
        x0=xyO(i,1);
        [t,x]=ode45(@(t,x) u(i,1)*t,tspan,x0);
        xyO(i,1)=x(end);
        %xyO(i,2)=xyO(i,2)+u(i,2);
        xyO(i,3)=atan2(u(i,2),u(i,1));
    
        y0=xyO(i,2);
        [t,y]=ode45(@(t,y) u(i,2)*t,tspan,y0);
        xyO(i,2)=y(end);
    end                  
end
%% INPUT PLOT

r=2;
c_map=lines(n_f);
for i=1:n_f
    center=[xyO_1(i,1)+r*cos(xyO_1(i,3)-xyO_1(i,4)/2),xyO_1(i,2)+r*sin(xyO_1(i,3)-xyO_1(i,4)/2);
            xyO_1(i,1),xyO_1(i,2);
            xyO_1(i,1)+r*cos(xyO_1(i,3)+xyO_1(i,4)/2),xyO_1(i,2)+r*sin(xyO_1(i,3)+xyO_1(i,4)/2)];
    theta=[xyO_1(i,3)-xyO_1(i,4)/2:pi/3600:xyO_1(i,3)+xyO_1(i,4)/2];
    x_trial=xyO_1(i,1)+r*cos(theta);
    y_trial=xyO_1(i,2)+r*sin(theta);
    plot(center(2,1),center(2,2),'bs');
    hold on;
    plot(center(:,1),center(:,2),x_trial,y_trial,'Color',c_map(i,:));
    grid on;
end

%% PLOTS
for i=1:n_f
    %figure;
    plot(f_x(1,i),f_y(1,i),'s');
    hold on;
    plot(f_x(:,i),f_y(:,i));
    hold on;
    plot(f_x(end,i),f_y(end,i),'o');
    grid on;
end
