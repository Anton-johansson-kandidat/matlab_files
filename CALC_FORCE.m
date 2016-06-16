%CALC_FORCE program, note that some parameters might differ

function [F,timeone,timetwo,visk,p,vmedeltillplot,mfp,vlagring] = CALC_FORCE(y,n,length,lambdany)

numberpeople=n; %Number of people
length_corridor=length; %The length of the corridor
fk = 1.6; %Force from the wall
he=10; %Defining lower wall
ht=12.5; %Defining upper wall
d=0.2; %Distance from the wall until they feel the force, quite large since you're always aware of the wall

timeone = zeros(1,numberpeople); %Used to calculate the time it takes for them to pass through the corridor
timetwo = zeros(1,numberpeople); %Used to calculate the time it takes for them to pass through the corridor

F = zeros(4*numberpeople,1); %This is the vector that contains all information
j=1; %This is used to create all the repulsive and attractive forces later on
vloopvariable=1; %Used as a loop variable


minussign=0; %Used to denote when we have passed half the population
personalsphere=0.55; %This is their personalsphere,inside it they feel forces from other people

aenemy=3.5; %This is used in the repelling force between individuals in different crowds
afriend = 2; %The constant of repulsion for people in the same crowd
b=personalsphere; %Constant used in repelling force
alpha1 = 0.5; %For the x-driving force
alpha2 = 8; %For the y-driving force
v0=1.5; %Their desired speed
lambda = lambdany; %The value of the perception anisotropy

%For viscosity
vlagring = zeros(1,numberpeople); %Used to store the velocities
mfplagring = zeros(1,numberpeople); %Used to store the mean free path, capital lambda


%This is the loop that creates the equations for all the persons
for i=1:4:4*numberpeople %increase with 4 each step since there are for equation for each person
    
    if(i>=4*numberpeople/2) %Checks if we have passed half the population
       minussign=1; 
    end
    
    drivingforcex = ((-1)^(minussign)*v0-y(i+1))/alpha1; %The driving forcex
    lowerwallrepulsion=0; %The lower wall repulsion
    higherwallrepulsion=0; %The upper wall repulsion

    if(abs(he-y(i+2))<d) %If they are within distance d from the wall they should feel the force
    lowerwallrepulsion = -fk*(1-d/(abs(y(i+2)-he))); %Repulsion from the wall
    end
    if (abs(ht-y(i+2))<d)   %If they are within distance d from the wall they should feel the force
    higherwallrepulsion = fk*(1-d/(abs(y(i+2)-ht))); %repulsion from thw wall
    end
    drivingforcey = alpha2*(-y(i+3)); %Calculates the driving forcey
    pedestrepulsivex=0; %The repulsive force in x-direction
    pedestrepulsivey=0; %The repulsive force in y-direction
    
    peopleinvicinity=0; %stores the number of people inside the radius of repulsion
    peopleinvicinityny=0;
    for k=1:4:4*numberpeople %This loop creates all the forces between the pedestrians hade 4*n-4
        theta_calculated=0; %says that the angle has not been calcualted yet
        if k ~= i %Check so the pedestrian wont exert forces on himself
            
            peddistance = sqrt((y(i+2)-y(k+2))^2+(y(i)-y(k))^2); %Distance between pedestrians
            

            
            %The code below calculates the variables of those who feel
            %each other across the boundary  
            if y(i)<-7.1 && minussign==1 %Check so they are close to the boundary and they are traveling to the left
                if (abs(y(i)-y(k))>14.2) %Check if the person is close to the other across the boundary
                   peddistance = sqrt((y(i+2)-y(k+2))^2+((7.5-abs(y(i)))+(7.5-abs(y(k))))^2);%Calculate the distance between them
                   theta = atan((y(i+2)-y(k+2))/((7.5-abs(y(i)))+(7.5-abs(y(k))))); %calculate the angle
                   theta_calculated = 1; %The angle does not need to be calculated down below now
                end
            end
            
             if y(i)>7.1 && minussign==0 %Check so they are close to the boundary and traveling to the rigth
                 if (abs(y(i)-y(k))>14.2) %Check so they are close to each other across the boundary
                   peddistance = sqrt((y(i+2)-y(k+2))^2+((7.5-abs(y(i)))+(7.5-abs(y(k))))^2); %Calculate the distance between the pedestrians across the boundary
                   theta = atan((y(i+2)-y(k+2))/((7.5-abs(y(i)))+(7.5-abs(y(k))))); %Calculate the angle
                   theta_calculated = 1;%The angle does not need to be calculated below
                end
             end
            %End of code for across the boundary
            %Stores the mean free path
            if (peddistance < personalsphere)
                mfplagring(vloopvariable) = mfplagring(vloopvariable) + peddistance;
            end
            if peddistance < personalsphere
                peopleinvicinityny=peopleinvicinityny+1; %Calculats the amount of people inside the personal radius
            end
            
            
            %The code below calculates the angle theta between pedestrians
            %Kanske endast borde räkna ut dessa om de befinner sig i
            %personalsphere?
            if theta_calculated==0
             if minussign ==0 && peddistance < personalsphere
                if(y(i)<y(k))
                   theta = atan((y(k+2)-y(i+2))/abs((y(i)-y(k)))); 
                else
                    theta =pi- atan((y(k+2)-y(i+2))/abs((y(i)-y(k))));
                end
                theta_calculated = 1;
               
             end
             if minussign ==1 && peddistance < personalsphere
                if(y(i)<y(k))
                   theta = pi-atan((y(k+2)-y(i+2))/(abs(y(i)-y(k)))); 
                else
                    theta = atan((y(k+2)-y(i+2))/(abs(y(i)-y(k))));
                end
                theta_calculated = 1;
             end
            end
            %End of code for theta
            
            
            
             %calculate the anisotropy
            if theta_calculated ==1
            anisotrop = lambda + (1-lambda)*(1-cos(pi-theta))*0.5;
            end
            %Sees how many are in close vicinity
            if peddistance < personalsphere
                peopleinvicinity=peopleinvicinity+1;
            end
            
            %Calculates the repulsive forces
            if (peddistance < personalsphere) && minussign==0 && k<2*numberpeople-1 %If they are in the left crowd and they feel a friend
               pedestrepulsivex = pedestrepulsivex - anisotrop*afriend*exp((0.5-peddistance)/b)*cos(theta); %Calculates force in x-direction
               pedestrepulsivey = pedestrepulsivey - anisotrop*afriend*exp((0.5-peddistance)/b)*sin(theta); %Calculates force in y-direction
            end
            if (peddistance < personalsphere) && minussign==0 && k>=2*numberpeople %If they are in the left crowd and they feel an enemy
                pedestrepulsivex = pedestrepulsivex - anisotrop*aenemy*exp((0.5-peddistance)/b)*cos(theta); %Calculates force in x-direction
               pedestrepulsivey = pedestrepulsivey - anisotrop*aenemy*exp((0.5-peddistance)/b)*sin(theta); %Calculates force in y-direction
            end
            if (peddistance < personalsphere) && minussign==1 && k<2*numberpeople-1 %If they are in the right crowd and they feel an enemy
               pedestrepulsivex = pedestrepulsivex + anisotrop*aenemy*exp((0.5-peddistance)/b)*cos(theta); %Calculates force in x-direction
               pedestrepulsivey = pedestrepulsivey - anisotrop*aenemy*exp((0.5-peddistance)/b)*sin(theta); %Calculates force in y-direction
            end
            if (peddistance < personalsphere) && minussign==1 && k>=2*numberpeople %If they are in the right crowd and they feel a friend
               pedestrepulsivex = pedestrepulsivex + anisotrop*afriend*exp((0.5-peddistance)/b)*cos(theta); %Calculates force in x-direction
               pedestrepulsivey = pedestrepulsivey - anisotrop*afriend*exp((0.5-peddistance)/b)*sin(theta); %Calculates force in y-direction
            end
            %End of repulsive forces
        end
            
    end
    
    if peopleinvicinity ~=0
    mfplagring(vloopvariable) = mfplagring(vloopvariable)/peopleinvicinityny;
    end
    j=j+4;
    density_factor=exp(-0.4*(peopleinvicinity-1)); %Calculates the density factor
    F(i:i+3,1) = [y(i+1);drivingforcex+density_factor*pedestrepulsivex;y(i+3);lowerwallrepulsion+higherwallrepulsion+drivingforcey+density_factor*pedestrepulsivey]; %The system of ODE:s
    
    %The code below
    vlagring(vloopvariable) = sqrt(y(i+1)^2+y(i+3)^2);
    
    %Slut på kod for visk
    
    
    
    
    %This belongs to the polarization
    if minussign ==1 %Checks which crowd we are on
    averageanglex(1,vloopvariable) = y(i+1); %This one is used in the polariation code
    averageangley(1,vloopvariable) = y(i+3); %This one is used in the polarization code
    vink(1,vloopvariable) = atan(y(i+3)/y(i+1)); %Calculate the angle
    end
    %End of polariation code
    vloopvariable =vloopvariable +1;
    
end

if sum(mfplagring~=0) == 0 %calculates the mean free path
    mfp = personalsphere;
else
mfp = sum(mfplagring)/sum(mfplagring~=0);
end
visk = 1/mean(vlagring)*1/mfp; %calculates the viscosity

vmedeltillplot = mean(vlagring); %The mean velocity

vxxx = sum(averageanglex)/(numberpeople/2);
vyyy = sum(averageangley)/(numberpeople/2);
vvektor = sqrt(vxxx^2+vyyy^2); %The veclosity vector is stored here

    
%detta hör till polariering
thetamedel = (sum(averageangley)/(numberpeople/2))/(sum(averageanglex)/(numberpeople/2)); %Calculate the average angle of motion
d = zeros(1,numberpeople/2); %The d used in the polarization
for r=1:numberpeople/2 %Loops over half the crowd
    d(1,r) = vink(1,r)-thetamedel;
    d(1,r) = rem(d(1,r),2*pi);
    
    d(1,r) = abs(d(1,r));
end
p = sum(d)/(numberpeople/2); %The polarization
    
end
