%Simulating single lane scenario
close all;
saftey_dist = 10;
comm_range = 50; % Range in which nodes can be connected to each other
dist_between_lanes = 3;
target_node = zeros(3,1);
avg_v2v = [];
avg_duration = [];
avg_num_same_neighbours = [];
for num_cars_per_lane = 100:50:500 %outer loop
    %Generating ID num for each node in a lane
    id_lane1 =randi([10001,10500],1,num_cars_per_lane);
    id_lane2 = randi([10501,11000],1,num_cars_per_lane);
    id_lane3 = randi([11001,11500],1,num_cars_per_lane);
    id_lane4 = randi([11501,12000],1,num_cars_per_lane);
    container_v2v = zeros(1,5);
    container_duration_same3nodes = zeros(1,5);
    container_same_neighbours = zeros(1,5);
    for iteration_count = 1:5
        duration_counter = 1;
        for time_interval = 0:0.1:600
            
            
            lane1_simulation = single_lane_simulation(num_cars_per_lane,1,id_lane1);
            lane2_simulation = single_lane_simulation(num_cars_per_lane,2,id_lane2);
            lane3_simulation = single_lane_simulation(num_cars_per_lane,3,id_lane3);
            lane4_simulation = single_lane_simulation(num_cars_per_lane,4,id_lane4);
            
            %Selecting a target node from lane1
            target_node = lane1_simulation(1:3,50);
            
            %Entry and exit at a randomly chosen ramp
            entry_ramp = randi([1,3]);
            exit_ramp = randi([1,3]);
            
            num_nodes_arriving = rand < 0.833; %arrival rate
            num_nodes_departing = num_nodes_arriving;
            
            entry_ramps_table = [1550 2550 4050];
            exit_ramps_table = [1500  2500 4000];
            
            current_entry_ramp_pos = entry_ramps_table(entry_ramp);
            current_exit_ramp_pos = exit_ramps_table(exit_ramp);
            
            safe_dist_entry_forward =current_entry_ramp_pos + 10;
            safe_dist_entry_back = current_entry_ramp_pos -10;
            
            %Starting entry at the 4th Lane
            back_lane4 = find(lane4_simulation(2,:)> safe_dist_entry_back);
            forward_lane4 = find(lane4_simulation(2,:)< safe_dist_entry_forward);
            
            %Starting entry at the 3rd Lane
            back_lane3 = find(lane3_simulation(2,:)> safe_dist_entry_back);
            forward_lane3 = find(lane3_simulation(2,:)< safe_dist_entry_forward);
            
            %Starting entry at the 2nd Lane
            back_lane2 = find(lane2_simulation(2,:)> safe_dist_entry_back);
            forward_lane2 = find(lane2_simulation(2,:)< safe_dist_entry_forward);
            
            %Starting entry at the 1st Lane
            back_lane1 = find(lane1_simulation(2,:)> safe_dist_entry_back);
            forward_lane1 = find(lane1_simulation(2,:)< safe_dist_entry_forward);
            
            %If no vehicle infront or backside within saftey distance,
            %enter the lane.Simultaneously make a node exit simulation.
            if isempty(back_lane4) && isempty(forward_lane4)
                lane_indicator = 4;
                lane4_simulation = exit_entry_ramps(lane4_simulation,current_entry_ramp_pos,current_exit_ramp_pos,...
                    safe_dist_entry_forward,safe_dist_entry_back,target_node,lane_indicator);
                
            elseif isempty(back_lane3) && isempty(forward_lane3)
                lane_indicator = 3;
                lane3_simulation = exit_entry_ramps(lane3_simulation,current_entry_ramp_pos,current_exit_ramp_pos,...
                    safe_dist_entry_forward,safe_dist_entry_back,target_node,lane_indicator);
                
            elseif isempty(back_lane2) && isempty(forward_lane2)
                lane_indicator = 2;
                lane2_simulation = exit_entry_ramps(lane2_simulation,current_entry_ramp_pos,current_exit_ramp_pos,...
                    safe_dist_entry_forward,safe_dist_entry_back,target_node,lane_indicator);
                
            elseif isempty(back_lane1) && isempty(forward_lane1)
                lane_indicator = 1;
                lane1_simulation = exit_entry_ramps(lane1_simulation,current_entry_ramp_pos,current_exit_ramp_pos,...
                    safe_dist_entry_forward,safe_dist_entry_back,target_node,lane_indicator);
            end
            
            %Lane changing
            [lane1_simulation,lane2_simulation,lane3_simulation,lane4_simulation] =...
                lane_changing(num_cars_per_lane,lane1_simulation,lane2_simulation,lane3_simulation,lane4_simulation,...
                saftey_dist);
            
            %Compute the range for v2v connectivity.
            lane1_comm_range = 50;
            lane2_comm_range = sqrt((comm_range)^2 - (dist_between_lanes)^2);
            lane3_comm_range = sqrt((comm_range)^2 - (2*dist_between_lanes)^2);
            lane4_comm_range = sqrt((comm_range)^2 - (3*dist_between_lanes)^2);
            
            %Find nodes in range of target node in lane1
            forward_range_lane1 = lane1_simulation(2,find(lane1_simulation(2,:) <= (target_node(2,1)+ lane1_comm_range) ));
            nodes_pos_for1 = forward_range_lane1(find(forward_range_lane1 >= (target_node(2,1))));
            
            forward_range_lane1_id = [];
            for k = 1:length(nodes_pos_for1)
                forward_range_lane1_id = [forward_range_lane1_id lane1_simulation(3,find(lane1_simulation(2,:) == nodes_pos_for1(k)))];
            end
            
            
            back_range_lane1 = lane1_simulation(2,find(lane1_simulation(2,:) >= (target_node(2,1)- lane1_comm_range)));
            nodes_pos_back1 = back_range_lane1(find(back_range_lane1 <= (target_node(2,1))));
            back_range_lane1_id = [];
            for k = 1:length(nodes_pos_back1)
                back_range_lane1_id = [back_range_lane1_id lane1_simulation(3,find(lane1_simulation(2,:) == nodes_pos_back1(k)))];
            end
            
            num_nodes_in_range1 = length(nodes_pos_for1)+length(nodes_pos_back1);
            
            %Find nodes in range of target node in lane2
            forward_range_lane2 = lane2_simulation(2,find(lane2_simulation(2,:) <= (target_node(2,1) + lane2_comm_range) ));
            nodes_pos_for2 = forward_range_lane2(find(forward_range_lane2 >= (target_node(2,1))));
            
            forward_range_lane2_id = [];
            for k = 1:length(nodes_pos_for2)
                forward_range_lane2_id = [forward_range_lane2_id lane2_simulation(3,find(lane2_simulation(2,:) == nodes_pos_for2(k)))];
            end
            
            back_range_lane2 = lane2_simulation(2,find(lane2_simulation(2,:) >= (target_node(2,1)- lane2_comm_range)));
            nodes_pos_back2 = back_range_lane2(find(back_range_lane2 <= (target_node(2,1))));
            
            back_range_lane2_id = [];
            for k = 1:length(nodes_pos_back2)
                back_range_lane2_id = [back_range_lane2_id lane2_simulation(3,find(lane2_simulation(2,:) == nodes_pos_back2(k)))];
            end
            
            num_nodes_in_range2 = length(nodes_pos_for2)+length(nodes_pos_back2);
            
            %Find nodes in range of target node in lane3
            forward_range_lane3 = lane3_simulation(2,find(lane3_simulation(2,:) <= (target_node(2,1) + lane3_comm_range) ));
            nodes_pos_for3 = forward_range_lane3(find(forward_range_lane3 >= (target_node(2,1))));
            
            forward_range_lane3_id = [];
            for k = 1:length(nodes_pos_for3)
                forward_range_lane3_id = [forward_range_lane3_id lane3_simulation(3,find(lane3_simulation(2,:) == nodes_pos_for3(k)))];
            end
            
            back_range_lane3 = lane3_simulation(2,find(lane3_simulation(2,:) >= (target_node(2,1)- lane3_comm_range)));
            nodes_pos_back3 = back_range_lane3(find(back_range_lane3 <= (target_node(2,1))));
            back_range_lane3_id = [];
            for k = 1:length(nodes_pos_back3)
                back_range_lane3_id = [back_range_lane3_id lane3_simulation(3,find(lane3_simulation(2,:) == nodes_pos_back3(k)))];
            end
            
            num_nodes_in_range3 = length(nodes_pos_for3)+length(nodes_pos_back3);
            
            %Find nodes in range of target node in lane4
            forward_range_lane4 = lane4_simulation(2,find(lane4_simulation(2,:) <= (target_node(2,1) + lane4_comm_range) ));
            nodes_pos_for4 = forward_range_lane4(find(forward_range_lane4 >= (target_node(2,1))));
            
            forward_range_lane4_id = [];
            for k = 1:length(nodes_pos_for4)
                forward_range_lane4_id = [forward_range_lane4_id lane4_simulation(3,find(lane4_simulation(2,:) == nodes_pos_for4(k)))];
            end
            
            back_range_lane4 = lane4_simulation(2,find(lane4_simulation(2,:) >= (target_node(2,1)- lane4_comm_range)));
            nodes_pos_back4 = back_range_lane4(find(back_range_lane4 <= (target_node(2,1))));
            
            back_range_lane4_id = [];
            for k = 1:length(nodes_pos_back4)
                back_range_lane4_id = [back_range_lane4_id lane4_simulation(3,find(lane4_simulation(2,:) == nodes_pos_back4(k)))];
            end
            num_nodes_in_range4 = length(nodes_pos_for4)+length(nodes_pos_back4);
            
            total_connected_nodes = num_nodes_in_range1 + num_nodes_in_range2 + num_nodes_in_range3 + num_nodes_in_range4;
            
            %Find any three randomly selected nodes which are in V2V range
            all_nodes_in_range = unique([forward_range_lane1_id forward_range_lane2_id forward_range_lane3_id forward_range_lane4_id back_range_lane1_id back_range_lane2_id back_range_lane3_id back_range_lane4_id]);
            if time_interval == 0
                all_nodes_in_range_beginning = all_nodes_in_range;
                arr = randi([1,length(all_nodes_in_range)],3);
                selected_index = arr(1,:);
                selected_nodes = all_nodes_in_range(selected_index);
            else
                node_present_array = [];
                for n = 1:length(all_nodes_in_range)
                    node_present = find(all_nodes_in_range(n) == selected_nodes);
                    node_present_array = [node_present_array node_present];
                end
                if length(node_present_array) ==3
                    duration_counter = duration_counter + 1;
                end
            end
            
            %To find avg number of same communication neighbours within
            %range of[10,60]
            if time_interval == 10
                node_repeated_array_10 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_10 = [node_repeated_array_10 node_repeated];
                end
            end
            if time_interval == 20
                node_repeated_array_20 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_20 = [node_repeated_array_20 node_repeated];
                end
            end
            if time_interval == 30
                node_repeated_array_30 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_30 = [node_repeated_array_30 node_repeated];
                end
            end
            if time_interval == 40
                node_repeated_array_40 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_40 = [node_repeated_array_40 node_repeated];
                end
            end
            if time_interval == 50
                node_repeated_array_50 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_50 = [node_repeated_array_50 node_repeated];
                end
            end
            if time_interval == 60
                node_repeated_array_60 =[];
                for n = 1:length(all_nodes_in_range)
                    node_repeated = find(all_nodes_in_range(n) == all_nodes_in_range_beginning);
                    node_repeated_array_60 = [node_repeated_array_60 node_repeated];
                end
            end
            
        end
        %Perform averaging of v2v connectivity.
        container_v2v(iteration_count) = total_connected_nodes;
        container_duration_same3nodes(iteration_count) = duration_counter*0.1;
        total_num_repeated_nodes = length(node_repeated_array_10) + length(node_repeated_array_20) + length(node_repeated_array_30) + length(node_repeated_array_40) + length(node_repeated_array_50) + length(node_repeated_array_60);
        container_same_neighbours(iteration_count) = total_num_repeated_nodes/6;
        
    end
    sprintf('Processing done for %s cars per lane',num2str(num_cars_per_lane))
    
    avg_v2v = [avg_v2v mean(container_v2v)];
    
    avg_duration = [avg_duration mean(container_duration_same3nodes)];
    avg_num_same_neighbours  =[avg_num_same_neighbours mean(container_same_neighbours)];
    
    %legend('Number of Cars in a lane','Num of nodes in comm range')
    
    
    
end
%Plots
num_car_range = 100:50:500;
figure(1)
plot(num_car_range,avg_v2v,'LineWidth',1)
axis([100 500 0 200])
s4 = sprintf('Average V2V Connectivity for %sm range',num2str(comm_range));
title(s4)
xlabel('Traffic Density')
s = sprintf('Num nodes in comm range of %s',num2str(comm_range));
ylabel(s)
grid on

figure(2)
plot(num_car_range,avg_duration,'LineWidth',1)
axis([100 500 0 200])
grid on

s3 = sprintf('Average Duration of Connectivity for %sm range',num2str(comm_range));
title(s3)
xlabel('Traffic Density')
s1 = sprintf('Avg duration of same 3 neighbours in seconds for %sm range',num2str(comm_range));
ylabel(s1)
grid on

figure(3)
plot(num_car_range,avg_num_same_neighbours,'LineWidth',1)
axis([100 500 0 200])

s2 = sprintf('Avg num of same comm neighbours for %sm range',num2str(comm_range));
title(s2)
xlabel('Traffic Density')
ylabel(s2)
grid on




