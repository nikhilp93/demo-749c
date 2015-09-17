%%Simulating single lane scenario excluding the lane changing and entry
%%exit ramps
function [vehicle_matrix] =single_lane_simulation(num_cars_per_lane,ind,id_lane)
saftey_dist = 10;

speed = zeros(1,500);
position = zeros(1,500);
speed(1:num_cars_per_lane) = randi([50,70],1,num_cars_per_lane);%speed is not yet converted to m/sec
%position(1:num_cars_per_lane) = randi([0,5000],1,num_cars_per_lane);
position(1:num_cars_per_lane) = randperm(5000,num_cars_per_lane);

vehicle = [speed ;position];
%Creating a new 2D matrix for storing the vehicle speed and position in a
%descending order.
vehicle_matrix = zeros(3,500);
vehicle_matrix(2,1:num_cars_per_lane) = sort(vehicle(2,1:num_cars_per_lane),'descend');

if ind == 1
    vehicle_matrix(3,1:num_cars_per_lane) = id_lane;
elseif ind == 2
    vehicle_matrix(3,1:num_cars_per_lane) = id_lane;
elseif ind == 3
    vehicle_matrix(3,1:num_cars_per_lane) = id_lane;
elseif ind == 4
    vehicle_matrix(3,1:num_cars_per_lane) = id_lane;
end

%Finding the corresponding speed of the vehicle for the sorted positions of
%the vehicles.
for i=1:num_cars_per_lane
    vehicle_matrix(1,i) = vehicle(1,find(vehicle(2,1:num_cars_per_lane)== vehicle_matrix(2,i)))*0.4470;%for converting to meter;
end

%Position diff gives us the distance between two vehicles in the lane.
position_diff = [];
position_diff = diff(vehicle_matrix(2,:));
position_diff = [0 position_diff];


%Car following model
beta = 0.75;
gamma = 0.0070104;
val = beta^2 + 4*gamma*saftey_dist;
ref_speed = ((-beta + sqrt(val))/(2*gamma))*2.23694;

%If the distance between follower and leader is less than Ds,reduce the followers speed to min value
for k=2:length(position_diff(1:num_cars_per_lane))
    if abs(position_diff(k)) <= saftey_dist
        vehicle_matrix(1,k) = min(vehicle_matrix(1,k-1),ref_speed);
    end
end



%calculating the new speed and acceleration
random_acceleration = -1 + 2*rand(1,1);
acceleration = randi([0,5],1);
for count=1:num_cars_per_lane
    
    vehicle_matrix(1,count) = vehicle_matrix(1,count) + random_acceleration*acceleration;
end


%Node reaching the boundary will randomly appear in the simulation
%region in the same lane.
if vehicle_matrix(2,1) > 5000
    
    random_pos = randi([0 5000]);
    location_overlap = find(vehicle_matrix(2,:) == random_pos);
    
    if isempty(location_overlap)
        vehicle_matrix(2,1) = random_pos; %vehicle2(2,1) - 5000;
    else
        while ~isempty(location_overlap)
            random_pos = randi([0 5000]);
            vehicle_matrix(2,1) = random_pos;
        end
    end    
end
end