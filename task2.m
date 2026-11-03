dt = 0.1;                 
T = 200;                  
t = 0:dt:T;
N = length(t);
lat0 = 12.9716;
lon0 = 77.5946;
x = zeros(1,N);
y = zeros(1,N);
x(1) = 0;
y(1) = 0;
speed = 3.5;              
altitude = zeros(1,N);
gps_noise = 0.5;
waypoints = [
    0    0;
    40   20;
    80   50;
    120  30;
    160  70;
    200  100];
num_waypoints = size(waypoints,1);
current_wp = 2;
wp_radius = 2;
gps_x = zeros(1,N);
gps_y = zeros(1,N);
latitude = zeros(1,N);
longitude = zeros(1,N);
flight_speed = zeros(1,N);
position_error = zeros(1,N);
for k = 1:N-1
    gps_x(k) = x(k) + gps_noise*randn;
    gps_y(k) = y(k) + gps_noise*randn;
    if current_wp <= num_waypoints
        target = waypoints(current_wp,:);
        dx = target(1) - gps_x(k);
        dy = target(2) - gps_y(k);
        distance = sqrt(dx^2 + dy^2);
        if distance < wp_radius

            current_wp = current_wp + 1;
        end
        if current_wp <= num_waypoints
            target = waypoints(current_wp,:);
            dx = target(1) - gps_x(k);
            dy = target(2) - gps_y(k);
            distance = sqrt(dx^2 + dy^2);
            if distance > 0
                vx = speed * dx/distance;
                vy = speed * dy/distance;
                x(k+1) = x(k) + vx*dt;
                y(k+1) = y(k) + vy*dt;
            else
                x(k+1) = x(k);
                y(k+1) = y(k);
            end
        else
            x(k+1) = x(k);
            y(k+1) = y(k);
        end
    else
        x(k+1) = x(k);
        y(k+1) = y(k);
    end
    altitude(k) = 20 + 2*sin(0.05*t(k));
    flight_speed(k) = sqrt((x(k+1)-x(k))^2+(y(k+1)-y(k))^2)/dt;
    position_error(k) = sqrt((x(k)-waypoints(min(current_wp,num_waypoints),1))^2+(y(k)-waypoints(min(current_wp,num_waypoints),2))^2);
end
latitude = lat0 + y/(111320);
longitude = lon0 + x/(111320*cosd(lat0));
figure('Name','Autonomous UAV Monitoring Dashboard');
subplot(2,2,1);
plot(waypoints(:,1),waypoints(:,2),'ro--','LineWidth',1.5);
hold on;
plot(x,y,'b','LineWidth',2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('UAV Navigation Path');
legend('Waypoints','UAV Path');
grid on;
subplot(2,2,2);
plot(t,altitude,'LineWidth',1.5);
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Altitude Monitoring');
grid on;
subplot(2,2,3);
plot(t,flight_speed,'LineWidth',1.5);
xlabel('Time (seconds)');
ylabel('Speed (m/s)');
title('UAV Speed Monitoring');
grid on;
subplot(2,2,4);
plot(t,position_error,'LineWidth',1.5);
xlabel('Time (seconds)');
ylabel('Distance (m)');
title('Waypoint Tracking Error');
grid on;
figure('Name','GPS Tracking');
plot(longitude,latitude,'b','LineWidth',2);
hold on;
plot(lon0 + waypoints(:,1)/(111320*cosd(lat0)), ...
     lat0 + waypoints(:,2)/111320, ...
     'ro--','LineWidth',1.5);
xlabel('Longitude');
ylabel('Latitude');
title('GPS Based UAV Tracking');
legend('UAV GPS Path','GPS Waypoints');
grid on;
fprintf('\n');
fprintf('AUTONOMOUS UAV SIMULATION RESULTS\n');
fprintf('Initial Latitude  : %.6f\n',lat0);
fprintf('Initial Longitude : %.6f\n',lon0);
fprintf('Maximum Altitude  : %.2f m\n',max(altitude));
fprintf('Average Speed     : %.2f m/s\n',mean(flight_speed));
fprintf('Total Distance    : %.2f m\n',sum(sqrt(diff(x).^2 + diff(y).^2)));
fprintf('Total Waypoints   : %d\n',num_waypoints);
disp('UAV Simulation Completed Successfully');