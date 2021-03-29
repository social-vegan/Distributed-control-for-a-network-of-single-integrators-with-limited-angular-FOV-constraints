clc; clear all; close all;
%% Input - Initial State of followers
n_f=input('Enter Number of moving vehicles or followers: ');
n_l=0;
xy=zeros(n_l,2);
xy_1=zeros(n_l,2);
xyO=zeros(n_f,4);
xyO_1=zeros(n_f,4);
disp('Enter initial global posture info (x, y, theta) for the N moving vehicles or followers');
for i=1:n_f 
    for j=1:3
        if j==1
            w1=sprintf('Enter x (for vehicle %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for vehicle %d) : ',i);
        end
        if j==3
            w1=sprintf('Enter theta [in degree](for vehicle %d) : ',i);
        end
        xyO(i,j)=input(w1);
        xyO_1(i,j)=xyO(i,j);
    end
end
xyO_1(:,3)=deg2rad(xyO_1(:,3));
xyO_1(:,4)=pi; % Half-Plane FOVs sensor omega=pi
%% Input - Initial State of static Leaders
clc;
n_l=input('Enter Number of Static vehicles or Leaders: ');
xy=zeros(n_l,2);
xy_1=zeros(n_l,2);

disp('Enter initial position info (x, y) for the M static vehicles or leaders');
for i=1:n_l 
    for j=1:2
        if j==1
            w1=sprintf('Enter x (for Target %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for Target %d) : ',i);
        end
        xy(i,j)=input(w1);
        xy_1(i,j)=xy(i,j);
    end
end
%% Consensus & Containment Analysis
clc;
n=n_f+n_l;
xyO=xyO_1;
xy=xy_1;
t_tot=40;
if n_l==0
    t_tot=500;
end
f_x=zeros(t_tot,n_f);
f_y=zeros(t_tot,n_f);
d=ceil((n-1)/2);
p_val=1;
%del=0.5;
for time=1:1:t_tot
    e=zeros(n_f,2);
    r1=zeros(n_f,2);
    r2=zeros(n_f,2);
    u=zeros(n_f,2);
    d_in=zeros(n_f,1);
    d_in_1=zeros(n_f,1);
    
    f_x(p_val,:)=xyO(:,1);
    f_y(p_val,:)=xyO(:,2);
    p_val=p_val+1;
    
    for i=1:n_f
        
        e(i,1)=cos(xyO(i,3));
        e(i,2)=sin(xyO(i,3));
        
        for j=1:n_l
            if ((xy(j,1)-xyO(i,1))*e(i,1)+(xy(j,2)-xyO(i,2))*e(i,2))>=0
                r1(i,1)=r1(i,1)+xy(j,1)-xyO(i,1);
                r1(i,2)=r1(i,2)+xy(j,2)-xyO(i,2);
                d_in(i)=d_in(i)+1;
            end
            
            if ((xy(j,1)-xyO(i,1))*e(i,1)+(xy(j,2)-xyO(i,2))*e(i,2))<=0
                r2(i,1)=r2(i,1)+xy(j,1)-xyO(i,1);
                r2(i,2)=r2(i,2)+xy(j,2)-xyO(i,2);
                d_in_1(i)=d_in_1(i)+1;
            end
        end

        for j=1:n_f
            if j~=i
                if ((xyO(j,1)-xyO(i,1))*e(i,1)+(xyO(j,2)-xyO(i,2))*e(i,2))>=0
                    r1(i,1)=r1(i,1)+xyO(j,1)-xyO(i,1);
                    r1(i,2)=r1(i,2)+xyO(j,2)-xyO(i,2);
                    d_in(i)=d_in(i)+1;
                end
                
                if ((xyO(j,1)-xyO(i,1))*e(i,1)+(xyO(j,2)-xyO(i,2))*e(i,2))<=0
                    r2(i,1)=r2(i,1)+xyO(j,1)-xyO(i,1);
                    r2(i,2)=r2(i,2)+xyO(j,2)-xyO(i,2);
                    d_in_1(i)=d_in_1(i)+1;
                end
            end
        end
        
        
        if d_in(i)>=d
            u(i,:)=r1(i,:);
        end
        
        if d_in(i)<d
            u(i,:)=r2(i,:);
        end
        
    end
            
    tspan=[0 0.1];
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

if n_l~=0
    [k,av] = convhull(xy);
    hold on;
    plot(xy(:,1),xy(:,2),'o')
    hold on;
    plot(xy(k,1),xy(k,2),'k');
end

%% OUTPUT PLOT

if n_l~=0
    [k,av] = convhull(xy);
    hold on;
    plot(xy(:,1),xy(:,2),'o')
    hold on;
    plot(xy(k,1),xy(k,2),'k');
end

for i=1:n_f
    %figure;
    plot(f_x(1,i),f_y(1,i),'s');
    hold on;
    plot(f_x(:,i),f_y(:,i));
    hold on;
    if n_l==0
        plot(f_x(end,i),f_y(end,i),'o');
    end
    if n_l~=0
        plot(f_x(end,i),f_y(end,i),'*');
    end
    grid on;
end
