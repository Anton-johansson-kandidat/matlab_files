%SOLVE_SYSTEM code
%If you have any qustions please contact me at erikantonjohansson@live.com
global h L time numberofpeople xspawn upperwall lowerwall lambda

upperwall = 12.5; %Defining upperwall
lowerwall = 10; %Defining lowerwall
xspawn = 10; %The distance they will spawn from the corridor
L=20; %Length of corridor
time=60; %Time of simulation
h=0.01; %Timestep
t=[0:h:time]; %Defining the timeinterval
numberofpeople=100; %number of people
lambda =1; %The value of the perception anisotropy
SIZE_OF_t = size(t); %Stores the amount of timesteps
storage_of_solution = zeros(4*numberofpeople,SIZE_OF_t(2)); %Stores all the information about the solution
v=zeros(1,numberofpeople); %Used to store velocities
xzero = zeros(1,4*numberofpeople); %Used to store solution as well


%Starting positions for left crowd
deltay=11.1; %Starting height
deltax =-L-xspawn-rand; %Starting x-position

for d=1:4:4*numberofpeople/2
        xzero(d) = deltax; %Sets the x-starting position
        xzero(d+2) = deltay; %Sets the y-starting position
        
        deltax = deltax + 0.6; %The step in x-direction
        deltay=deltay+rand; %The y-psotions is randomized
        if deltax >-14 %When to stop the x-position spawn
           deltax=-L-xspawn+rand; %Move the starting position back
        end
        if deltay >12.3 %When to stop the y-psotion spawn
           deltay=10+rand; %Move the starting position back
        end
    
       
end
%End of starting position for left crowd

%Starting positions for right crowd
deltay=11; %starting height for the y-position
deltax =L+xspawn+rand; %starting x-position
for d=4*numberofpeople/2+1:4:4*numberofpeople-2
        xzero(d) = deltax; %Sets the x-starting position
        xzero(d+2) = deltay; %Sets the y-starting position
        deltax = deltax - 0.6; %Takes one step forward in x
        deltay=deltay+rand; %Takes a step in y
        if deltax <14 %When to stop advancing in x
           deltax=L+xspawn+rand; %Resets the x-value
        end
        if deltay >12.3 %When to stop in y
           deltay=10+rand; %Reset the y-value
        end
end
%End of starting position for right crowd

timeett=zeros(10,numberofpeople);%Used to store the time when they arrive in the corridor
timetva=zeros(10,numberofpeople);%Used to store the time when they leave in the corridor
k=1; %Loop variable
j=1; %Loop variable
tidvariab1 =ones(1,numberofpeople); %Stores the time when they enter the periodic corridor
tidvariab2 =ones(1,numberofpeople); %Stores the time when they leave the peridoci corridor
visclagring = zeros(1,SIZE_OF_t(2)); %Stores the lane viscosity
plagring = zeros(1,SIZE_OF_t(2)); %Stores the polarization values
storage_of_solution = zeros(4*numberofpeople,SIZE_OF_t(2)); %Stores the solution to the Runge-Kutta method


lambdalagring = zeros(1,SIZE_OF_t(2));
spaceviscv = zeros(SIZE_OF_t(2),numberofpeople);


for i=0:h:time
    j=1;
    storage_of_solution(:,k) = xzero'; %Stores the solution vector each round so that it can be plotted later on
    %Runge-Kutta method
    [k1,time1,time2,visc,p,spaceviscvek] = CALC_FORCE(xzero,numberofpeople,L,lambda);
    [k2,time1,time2,visc,p,spaceviscvek] = CALC_FORCE(xzero+h*k1'/2,numberofpeople,L,lambda);
    [k3,time1,time2,visc,p,spaceviscvek] = CALC_FORCE(xzero+h*k2'/2,numberofpeople,L,lambda);
    [k4,time1,time2,visc,p,spaceviscvek] = CALC_FORCE(xzero+h*k3',numberofpeople,L,lambda);%Calls on the system of ODE:S
    xzero = xzero+h*(k1'+2*k2'+2*k3'+k4')/6'; %Runge-kutta method method
    plagring(k)=p; %Stores the polarization
    visclagring(k) = visc; %Stores part of the lane viscosity

    spaceviscv(k,:) = spaceviscvek; %Stores the space-dependant viscosity index values
    %The code below is used for the periodic boundary conditions
   for d=1:4:4*numberofpeople/2
         if xzero(1,d)>6.4 %Periodic corridor BC
             xzero(1,d)=-6.4; %Resets the x-position to the other side of the corridor
             timeett(tidvariab1(1,j),j)=k; %Stores the time they leave the periodic corridor
           tidvariab1(1,j) = tidvariab1(1,j)+1; %Says that we have calculated the value of the time
         end
         j=j+1; %Increases a loop-variable
   end
   for d=2*numberofpeople+1:4:4*numberofpeople
         if xzero(1,d)<-6.4 %Periodic corridor BC
            xzero(1,d)=6.4; %Resets the x-position value
            timeett(tidvariab1(1,j),j)=k; %Stores the time they leave the corridor
            tidvariab1(1,j) = tidvariab1(1,j)+1; % Says that we have calcualted the value of the time
         end
         j=j+1; %Increases a loop-variable
   end 
   %End of boundary conditions code
    k=k+1; %Increases a loop-variable
    
end
storage_of_solution = storage_of_solution'; %Transposes the storage vector



%plot for spaceviscosity
%start of code
[X,Y] = meshgrid(-7:0.1:7,9.5:0.1:13); %Creates the grid which is used to plot the space-dependant viscosity index

Space_dependent_viscosity=0; %Sets the function value to 0 first for the space-dependant viscosity index



%The code Below plots the space-dependent viscosity index and the
%correspond sparital configuartion of pedestrians
k=1;
time_for_plot_SPV = 4000;
for SPV_loop_vairable=1:numberofpeople
Space_dependent_viscosity = Space_dependent_viscosity +  1/(spaceviscv(time_for_plot_SPV,SPV_loop_vairable)+0.2)*exp(-((X-storage_of_solution(time_for_plot_SPV,k)).^2+(Y-storage_of_solution(time_for_plot_SPV,k+2)).^2));    
k=k+4;
end
figure;
pcolor(X,Y,Space_dependent_viscosity); shading interp;
%The code below plots the spatial configuaration
figure;
for b=1:4:4*numberofpeople/2
plot(storage_of_solution(time_for_plot_SPV,b),storage_of_solution(time_for_plot_SPV,b+2),'ro')
axis([-6 6 lowerwall-0.5 upperwall+0.5])
hold on
end
for b=4*numberofpeople/2+1:4:4*numberofpeople
plot(storage_of_solution(time_for_plot_SPV,b),storage_of_solution(time_for_plot_SPV,b+2),'bx')
axis([-6 6 lowerwall-0.5 upperwall+0.5])
hold on
end
plot([-L/2-20,L/2+20],[upperwall,upperwall],'k','LineWidth',4)
    hold on
    plot([-L/2-20,L/2+20],[lowerwall,lowerwall],'k','LineWidth',4) 
    hold on
ylabel('y (m)')
xlabel('x (m)')
%End of code for space-dependent viscosity index

%The code below is used for the viscosity and polarization

%Used to calculate the mean polarization
viscplot = visclagring(2:end);
tplot = t(200:end-200);
ptidsmedel = zeros(1,SIZE_OF_t(2));
pepsilon = zeros(1,SIZE_OF_t(2)-399);



plagringjamvikt = plagring(350);
for u=1:400
plagring(u) = plagringjamvikt;
end

for j=2:SIZE_OF_t(2)  
ptidsmedel(j) = sum(plagring((2:j)))/(sum(plagring((2:j))>0));
end
k=1;
for i=400:SIZE_OF_t(2)-199
    for o=i-199:i+199
pepsilon(k) = pepsilon(k)+plagring(o);
    end
    pepsilon(k)=pepsilon(k)/400;
k=k+1;
end
%End of code for polarization mean


vmedel = sum(visclagring((SIZE_OF_t(2)-1)/2:end))/(sum(visclagring((SIZE_OF_t(2)-1)/2:end)>0));
vmedelny = visclagring(200:end-200);
%To plot mean polarization and the lane viscosity index. Remove the comment
%below
%{
plot(tplot,pepsilon)
ylabel('P(t)')
xlabel('t (s)')
axis([0 60 0 0.1])
lambda = lambda-0.25;
%}

%{
viskositeten = vmedelny.*pepsilon;
figure;
plot(tplot,viskositeten)
ylabel('\mu (t)')
xlabel('t (s)')
axis([0 time-5 0 0.06])
%}


