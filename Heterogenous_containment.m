clc; clear all;
%% Input - Initial State of Vehicles
nl=input('Enter Number of Leaders: ');
xyl=zeros(nl,2);
xyl_1=zeros(nl,2);
disp('Enter initial global posture info (x, y) for the N vehicles');
for i=1:nl 
    for j=1:2
        if j==1
            w1=sprintf('Enter x (for vehicle %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for vehicle %d) : ',i);
        end
        xyl(i,j)=input(w1);
        xyl_1(i,j)=xyl(i,j);
    end
end


nf=input('Enter Number of Followers: ');
xyf=zeros(nf,4);
xyf_1=zeros(nf,4);
disp('Enter initial global posture info (x, y, theta, omega) for the N vehicles');
for i=1:nf 
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
            w1=sprintf('Enter omega [in radians](for vehicle %d) : ',i);
        end
        xyf(i,j)=input(w1);
        xyf_1(i,j)=xyf(i,j);
    end
end
xyf_1(:,3)=deg2rad(xyf_1(:,3));
%%
clc;
xyf=xyf_1;
f_x=zeros(1000,nf);
f_y=zeros(1000,nf);
k = 0.181;
e = 1.701;
u0 = k;
w0 = 1;%3*pi*k/e;
u = zeros(nf,2);
p_val = 1;
r1 = zeros(nl,nf);
r2 = zeros(nl,nl);
r3 = zeros(nf,nf);

for i=1:nl
    for j=1:nf
        r1(i,j) = norm(xyl(i,1:2)-xyf(j,1:2));
    end
end
R1 = max(max(r1));
for i=1:nl
    for j=1:nl
        if j~=i
            r2(i,j) = norm(xyl(i,1:2)-xyl(j,1:2));
        end
    end
end
R2 = max(max(r2));
for i=1:nf
    for j=1:nf
        if j~=i
            r3(i,j) = norm(xyf(i,1:2)-xyf(j,1:2));
        end
    end
end
R3 = max(max(r3));
R4 = [R1 R2 R3];
R = max(R4);

for time=1:1:1000
    f_x(p_val,:)=xyf(:,1);
    f_y(p_val,:)=xyf(:,2);
    p_val=p_val+1;
    
    for i=1:nf
        %r1 = zeros(nf,2);
        for j=1:nl
            if norm(xyf(i,1:2)-xyl(j,1:2)) > e
                %r1(i) = r1(i)+ (xyl(i,1:2)-xyf(j,1:2));
                u(i,:) = u(i,:)+(xyl(j,1:2)-xyf(i,1:2));               
            end
        end
        u(i,:)=u(i,:)*(k/((nl+nf-1)*R));
        
    end
    
    tspan=[0 1];
    for i=1:nf
        x0=xyf(i,1);
        [t,x]=ode45(@(t,x) u(i,1)*t,tspan,x0);
        xyf(i,1)=x(end);
        %xyO(i,2)=xyO(i,2)+u(i,2);
        xyf(i,3)=atan2(u(i,2),u(i,1));
    
        y0=xyf(i,2);
        [t,y]=ode45(@(t,y) u(i,2)*t,tspan,y0);
        xyf(i,2)=y(end);
    end
                    
end
%%
%{
[k,av] = convhull(f_x(end,:),f_y(end,:));
hold on;
plot(f_x(end,k),f_y(end,k),'k');
%}

xylO = zeros(nl+1,2);
xylO(1:end-1,:) = xyl(:,:);
xylO(end,:) = xyl(1,:);
plot(xylO(:,1),xylO(:,2),'-o');
hold on

for i=1:nf
    %figure;
    plot(f_x(1,i),f_y(1,i),'s');
    hold on;
    plot(f_x(:,i),f_y(:,i));
    hold on;
    plot(f_x(end,i),f_y(end,i),'o');
    grid on;
end
