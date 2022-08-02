clear all
close all
clc
tic
%% Coordinates
%Lower Wishbone
Xl = [-122.58;-10;100]; 
Yl = [-210;-511.51;-210]; % inner-front , outer , inner-rear
Zl = [115.69;131;140]; % inner-front , outer , inner-rear
%Upper Wishbone
Xu = [-100;15.306;100]; 
Yu = [-240;-483;-240]; % inner-front , outer  , inner-rear
Zu = [313.36;328;279.26]; % inner-front , outer , inner-rear
%Wheel Center
X_wc = [0];
Y_wc = [-556.14]; 
Z_wc = [220.9];
%Wheel Top
X_wct = X_wc;
Y_wct = Y_wc;
wheelradius = 220.9;
Z_wct = Z_wc + (wheelradius);
%Tierod
X_tie = [80;58.308]; %inner , outer
Y_tie = [-175;-500.36]; %inner , outer
Z_tie = [180;182.3]; %inner , outer

plot3(Xl,Yl,Zl,'b');hold on;
plot3(Xl,Yl,Zl,'ko')
plot3(Xu,Yu,Zu,'b')
plot3(Xu,Yu,Zu,'ko')
plot3(X_tie,Y_tie,Z_tie,'b')
plot3(X_tie,Y_tie,Z_tie,'ko')
plot3([Xu(2) Xl(2)],[Yu(2) Yl(2)],[Zu(2) Zl(2)],'k')
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
title('Suspension Geometry')

%% Calculating the length of front and aft link
lca_front = sqrt(((Xl(2)-Xl(1))^2) + ((Yl(2)-Yl(1))^2) + ((Zl(2)-Zl(1))^2));
lca_rear = sqrt(((Xl(2)-Xl(3))^2) + ((Yl(2)-Yl(3))^2) + ((Zl(2)-Zl(3))^2));
uca_front = sqrt(((Xu(2)-Xu(1))^2) + ((Yu(2)-Yl(1))^2) + ((Zu(2)-Zu(1))^2));
uca_rear = sqrt(((Xu(2)-Xu(3))^2) + ((Yu(2)-Yu(3))^2) + ((Zu(2)-Zu(3))^2));
upright = sqrt(((Xu(2)-Xl(2))^2) + ((Yu(2)-Yl(2))^2) + ((Zu(2)-Zl(2))^2));
tierod = sqrt(((X_tie(2)-X_tie(1))^2) + ((Y_tie(2)-Y_tie(1))^2) + ((Z_tie(2)-Z_tie(1))^2));
wc_uca = sqrt(((X_wc-Xu(2))^2) + ((Y_wc-Yu(2))^2) + ((Z_wc-Zu(2))^2));
wc_lca = sqrt(((X_wc-Xl(2))^2) + ((Y_wc-Yl(2))^2) + ((Z_wc-Zl(2))^2));
wc_tierod = sqrt(((X_wc-X_tie(2))^2) + ((Y_wc-Y_tie(2))^2) + ((Z_wc-Z_tie(2))^2));
tierod_lca = sqrt(((X_tie(2)-Xl(2))^2) + ((Y_tie(2)-Yl(2))^2) + ((Z_tie(2)-Zl(2))^2));
tierod_uca = sqrt(((X_tie(2)-Xu(2))^2) + ((Y_tie(2)-Yu(2))^2) + ((Z_tie(2)-Zu(2))^2));
wct_uca = sqrt(((X_wct-Xu(2))^2) + ((Y_wct-Yu(2))^2) + ((Z_wct-Zu(2))^2));
wct_lca = sqrt(((X_wct-Xl(2))^2) + ((Y_wct-Yl(2))^2) + ((Z_wct-Zl(2))^2));
wct_tierod = sqrt(((X_wct-X_tie(2))^2) + ((Y_wct-Y_tie(2))^2) + ((Z_wct-Z_tie(2))^2));

syms x_l y_l x_u y_u z_u x_tie y_tie z_tie

ind = 1;
min_zl = Zl(2)-30;
max_zl = Zl(2)+30;
h        = waitbar(0,'Solving','Name','SD_Kinematics');
for z_l = min_zl:10:max_zl
eqn_lca_f = (x_l-Xl(1))^2 + (y_l-Yl(1))^2 + (z_l-Zl(1))^2 - lca_front^2 == 0;
eqn_lca_r = (x_l-Xl(3))^2 + (y_l-Yl(3))^2 + (z_l-Zl(3))^2 - lca_rear^2 == 0;
eqn_upright = (x_u-x_l)^2 + (y_u-y_l)^2 + (z_u-z_l)^2 - upright^2 == 0;
eqn_uca_f = (x_u-Xu(1))^2 + (y_u-Yl(1))^2 + (z_u-Zu(1))^2 - uca_front^2 == 0;
eqn_uca_r = (x_u-Xu(3))^2 + (y_u-Yu(3))^2 + (z_u-Zu(3))^2 - uca_rear^2 == 0;
eqn_tierod = ((x_tie-X_tie(1))^2) + ((y_tie-Y_tie(1))^2) + ((z_tie-Z_tie(1))^2) - (tierod^2) == 0;
eqn_tierod_lca = ((x_tie-x_l)^2) + ((y_tie-y_l)^2) + ((z_tie-z_l)^2) - (tierod_lca^2) == 0;
eqn_tierod_uca = ((x_tie-x_u)^2) + ((y_tie-y_u)^2) + ((z_tie-z_u)^2) - (tierod_uca^2) == 0;

eqns = [eqn_lca_f,eqn_lca_r,eqn_uca_f,eqn_uca_r,eqn_upright,eqn_tierod,eqn_tierod_lca,eqn_tierod_uca];

Out = solve(eqns,[x_l,y_l,x_u,y_u,z_u,x_tie,y_tie,z_tie]);
x_lower(ind,:) = double(Out.x_l);
y_lower(ind,:) = (double(Out.y_l));
z_lower(ind,:) = z_l;
x_upper(ind,:) = double(Out.x_u);
y_upper(ind,:) = (double(Out.y_u));
z_upper(ind,:) = double(Out.z_u);
x_tierod(ind,:) = double(Out.x_tie);
y_tierod(ind,:) = double(Out.y_tie);
z_tierod(ind,:) = double(Out.z_tie);

ind = ind+1;
waitbar((z_l/max_zl)/1,h,['Travel = ' num2str(z_l,'%4.1f') 'mm'])
end
hold on
close(h)
%% Finding the right solution (temp)
[mid mid] = min(abs(z_lower - Zl(2)));
inx = ((x_upper(mid,:)-Xu(2))<0.001) .* ((y_upper(mid,:)-Yu(2))<0.001) .* ((z_upper(mid,:)-Zu(2))<0.001);
inx = find(inx==1);
i = inx(1);
x_lower = x_lower(:,i);
y_lower = y_lower(:,i);
% z_lower = z_lower(:,i);
x_upper = x_upper(:,i);
y_upper = y_upper(:,i);
z_upper = z_upper(:,i);
x_tierod = x_tierod(:,i);
y_tierod = y_tierod(:,i);
z_tierod = z_tierod(:,i);

plot3(x_lower,y_lower,z_lower,'*')
plot3(x_upper,y_upper,z_upper,'*')
plot3(x_tierod,y_tierod,z_tierod,'*')

syms x_wc y_wc z_wc x_wct y_wct z_wct
for i = 1:length(x_lower)
    
eqn_wc_uca = ((x_wc-x_upper(i))^2) + ((y_wc-y_upper(i))^2) + ((z_wc-z_upper(i))^2) - (wc_uca^2) == 0;
eqn_wc_lca = ((x_wc-x_lower(i))^2) + ((y_wc-y_lower(i))^2) + ((z_wc-z_lower(i))^2) - (wc_lca^2) == 0;
eqn_wc_tierod = ((x_wc-x_tierod(i))^2) + ((y_wc-y_tierod(i))^2) + ((z_wc-z_tierod(i))^2) - (wc_tierod^2) == 0;
eqn_wct_uca = ((x_wct-x_upper(i))^2) + ((y_wct-y_upper(i))^2) + ((z_wct-z_upper(i))^2) - (wct_uca^2) == 0;
eqn_wct_lca = ((x_wct-x_lower(i))^2) + ((y_wct-y_lower(i))^2) + ((z_wct-z_lower(i))^2) - (wct_lca^2) == 0;
eqn_wct_tierod = ((x_wct-x_tierod(i))^2) + ((y_wct-y_tierod(i))^2) + ((z_wct-z_tierod(i))^2) - (wct_tierod^2) == 0;

eqn = [eqn_wc_uca,eqn_wc_lca,eqn_wc_tierod];
Out = solve(eqn,[x_wc,y_wc,z_wc]);
x_center(i,:) = double(Out.x_wc);
y_center(i,:) = double(Out.y_wc);
z_center(i,:) = double(Out.z_wc);

eqn = [eqn_wct_uca,eqn_wct_lca,eqn_wct_tierod];
Out1 = solve(eqn,[x_wct,y_wct,z_wct]);
x_centert(i,:) = double(Out1.x_wct);
y_centert(i,:) = double(Out1.y_wct);
z_centert(i,:) = double(Out1.z_wct);
end
[mid mid] = min(abs(z_lower - Zl(2)));
inx = ((x_center(mid,:)-X_wc)<0.001) .* ((y_center(mid,:)-Y_wc)<0.001) .* ((z_center(mid,:)-Z_wc)<0.001);
inx = find(inx==1);
i = inx(1);
x_center = x_center(:,i);
y_center = y_center(:,i);
z_center = z_center(:,i);
[mid mid] = min(abs(z_lower - Zl(2)));
inx = ((x_centert(mid,:)-X_wct)<0.001) .* ((y_centert(mid,:)-Y_wct)<0.001) .* ((z_centert(mid,:)-Z_wct)<0.001);
inx = find(inx==1);
i = inx(1);
x_centert = x_centert(:,i);
y_centert = y_centert(:,i);
z_centert = z_centert(:,i);
plot3(x_center,y_center,z_center,'*')
plot3(X_wc,Y_wc,Z_wc,'k*')
plot3(x_centert,y_centert,z_centert,'*')
plot3(X_wct,Y_wct,Z_wct,'k*')

%% Post Processing
ydist = y_centert - y_center;
zdist = z_centert - z_center;
camber = atand(ydist./zdist) + (-1*(atand((y_lower(mid)-y_upper(mid))/(z_lower(mid)-z_upper(mid)))));
% x_dist = x_tierod - x_center;
% y_dist = y_tierod - y_center;
% toe = atand(y_dist./x_dist) - (atand(y_dist(mid)/x_dist(mid)));
figure;plot(z_lower - z_lower(mid),camber)
solver_time = toc;
% grid on;xlabel('Lower Wishbone Outer balljoint Z travel');ylabel('Camber Angle');