%This function implements the Lane changing model
function [lane1_simulation,lane2_simulation,lane3_simulation,lane4_simulation] =...
    lane_changing(num_cars_per_lane,lane1_simulation,lane2_simulation,lane3_simulation,lane4_simulation,...
    saftey_dist)
max_len_lane = 500;
%lane changing from Lane1 to lane 2
for car_count = 1:num_cars_per_lane
    
    [value,index] = min(lane2_simulation(2,find(lane2_simulation(2,:) >lane1_simulation(2,car_count)))) ;
    empty_slot_ahead = value-10;
    c = max(lane2_simulation(2,find(lane2_simulation(2,:) < lane1_simulation(2,car_count))));
    empty_slot_behind = c + 10;
    if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
        if empty_slot_ahead > (lane1_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane1_simulation(2,car_count)-saftey_dist)
            
            temp = [];
            temp1 = [];
            temp = [lane2_simulation(1,1:index) 0 lane2_simulation(1,index+1:max_len_lane-1)];
            lane2_simulation(1,1:length(temp)) = temp;
            temp = [lane2_simulation(2,1:index) 0 lane2_simulation(2,index+1:max_len_lane-1)];
            lane2_simulation(2,1:length(temp)) = temp;
            temp = [lane2_simulation(3,1:index) 0 lane2_simulation(3,index+1:max_len_lane-1)];
            lane2_simulation(3,1:length(temp)) = temp;
            %placing position,speed and unique vehicle ID
            %from lane1 car into lane2
            lane2_simulation(1:3,index+1) = lane1_simulation(1:3,car_count);
            %Removing the shifting node from lane1
            temp1 = [lane1_simulation(1,1:car_count-1) lane1_simulation(1,car_count+1:max_len_lane-1)];
            lane1_simulation(1,1:length(temp1)) = temp1;
            temp1 = [lane1_simulation(2,1:car_count-1) lane1_simulation(2,car_count+1:max_len_lane-1)];
            lane1_simulation(2,1:length(temp1)) = temp1;
            temp1 = [lane1_simulation(3,1:car_count-1) lane1_simulation(3,car_count+1:max_len_lane-1)];
            lane1_simulation(3,1:length(temp1)) = temp1;
        end
    end
end
%lane changing from lane2 to either lane1 or lane3
for car_count = 1:num_cars_per_lane
    
    
    if rand < 0.5 % Go to lane1
        [value,index] = min(lane1_simulation(2,find(lane1_simulation(2,:) >lane2_simulation(2,car_count)))) ;
        empty_slot_ahead = value-10;
        c = max(lane1_simulation(2,find(lane1_simulation(2,:) < lane2_simulation(2,car_count))));
        empty_slot_behind = c + 10;
        
        if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
            if empty_slot_ahead > (lane2_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane2_simulation(2,car_count)-saftey_dist)
                temp = [];
                temp1 = [];
                temp = [lane1_simulation(1,1:index) 0 lane1_simulation(1,index+1:max_len_lane-1)];
                lane1_simulation(1,1:length(temp)) = temp;
                temp = [lane1_simulation(2,1:index) 0 lane1_simulation(2,index+1:max_len_lane-1)];
                lane1_simulation(2,1:length(temp)) = temp;
                temp = [lane1_simulation(3,1:index) 0 lane1_simulation(3,index+1:max_len_lane-1)];
                lane1_simulation(3,1:length(temp)) = temp;
                %placing position,speed and unique vehicle ID
                %from lane2 car into lane1
                lane1_simulation(1:3,index+1) = lane2_simulation(1:3,car_count);
                %Removing the shifting node from lane2
                temp1 = [lane2_simulation(1,1:car_count-1) lane2_simulation(1,car_count+1:max_len_lane-1)];
                lane2_simulation(1,1:length(temp1)) = temp1;
                temp1 = [lane2_simulation(2,1:car_count-1) lane2_simulation(2,car_count+1:max_len_lane-1)];
                lane2_simulation(2,1:length(temp1)) = temp1;
                temp1 = [lane2_simulation(3,1:car_count-1) lane2_simulation(3,car_count+1:max_len_lane-1)];
                lane2_simulation(3,1:length(temp1)) = temp1;
            end
        end
    else
        %Go to lane3
        [value,index] = min(lane3_simulation(2,find(lane3_simulation(2,:) >lane2_simulation(2,car_count)))) ;
        empty_slot_ahead = value-10;
        c = max(lane3_simulation(2,find(lane3_simulation(2,:) < lane2_simulation(2,car_count))));
        empty_slot_behind = c + 10;
        if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
            if empty_slot_ahead > (lane2_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane2_simulation(2,car_count)-saftey_dist)
                temp = [];
                temp1 = [];
                temp = [lane3_simulation(1,1:index) 0 lane3_simulation(1,index+1:max_len_lane-1)];
                lane3_simulation(1,1:length(temp)) = temp;
                temp = [lane3_simulation(2,1:index) 0 lane3_simulation(2,index+1:max_len_lane-1)];
                lane3_simulation(2,1:length(temp)) = temp;
                temp = [lane3_simulation(3,1:index) 0 lane3_simulation(3,index+1:max_len_lane-1)];
                lane3_simulation(3,1:length(temp)) = temp;
                lane3_simulation(1:3,index+1) = lane2_simulation(1:3,car_count);
                %Removing the shifting node from lane2
                temp1 = [lane2_simulation(1,1:car_count-1) lane2_simulation(1,car_count+1:max_len_lane-1)];
                lane2_simulation(1,1:length(temp1)) = temp1;
                temp1 = [lane2_simulation(2,1:car_count-1) lane2_simulation(2,car_count+1:max_len_lane-1)];
                lane2_simulation(2,1:length(temp1)) = temp1;
                temp1 = [lane2_simulation(3,1:car_count-1) lane2_simulation(3,car_count+1:max_len_lane-1)];
                lane2_simulation(3,1:length(temp1)) = temp1;
            end
        end
    end
end
%Changing from lane3 to either lane2 or lane4
for car_count = 1:num_cars_per_lane
    if rand < 0.5 % Go to lane2
        [value,index] = min(lane2_simulation(2,find(lane2_simulation(2,:) >lane3_simulation(2,car_count)))) ;
        empty_slot_ahead = value-10;
        c = max(lane2_simulation(2,find(lane2_simulation(2,:) < lane3_simulation(2,car_count))));
        empty_slot_behind = c + 10;
        if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
            if empty_slot_ahead > (lane3_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane3_simulation(2,car_count)-saftey_dist)
                temp = [];
                temp1 = [];
                temp = [lane2_simulation(1,1:index) 0 lane2_simulation(1,index+1:max_len_lane-1)];
                lane2_simulation(1,1:length(temp)) = temp;
                temp = [lane2_simulation(2,1:index) 0 lane2_simulation(2,index+1:max_len_lane-1)];
                lane2_simulation(2,1:length(temp)) = temp;
                temp = [lane2_simulation(3,1:index) 0 lane2_simulation(3,index+1:max_len_lane-1)];
                lane2_simulation(3,1:length(temp)) = temp;
                lane2_simulation(1:3,index+1) = lane3_simulation(1:3,car_count);
                %Removing the lane shifting node from lane3
                temp1 = [lane3_simulation(1,1:car_count-1) lane3_simulation(1,car_count+1:max_len_lane-1)];
                lane3_simulation(1,1:length(temp1)) = temp1;
                temp1 = [lane3_simulation(2,1:car_count-1) lane3_simulation(2,car_count+1:max_len_lane-1)];
                lane3_simulation(2,1:length(temp1)) = temp1;
                temp1 = [lane3_simulation(3,1:car_count-1) lane3_simulation(3,car_count+1:max_len_lane-1)];
                lane3_simulation(3,1:length(temp1)) = temp1;
            end
        end
    else
        %Go to lane4
        [value,index] = min(lane4_simulation(2,find(lane4_simulation(2,:) >lane3_simulation(2,car_count)))) ;
        empty_slot_ahead = value-10;
        c = max(lane4_simulation(2,find(lane4_simulation(2,:) < lane3_simulation(2,car_count))));
        empty_slot_behind = c + 10;
        if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
            if empty_slot_ahead > (lane3_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane3_simulation(2,car_count)-saftey_dist)
                temp = [];
                temp1 = [];
                temp = [lane4_simulation(1,1:index) 0 lane4_simulation(1,index+1:max_len_lane-1)];
                lane4_simulation(1,1:length(temp)) = temp;
                temp = [lane4_simulation(2,1:index) 0 lane4_simulation(2,index+1:max_len_lane-1)];
                lane4_simulation(2,1:length(temp)) = temp;
                temp = [lane4_simulation(3,1:index) 0 lane4_simulation(3,index+1:max_len_lane-1)];
                lane4_simulation(3,1:length(temp)) = temp;
                lane4_simulation(1:3,index+1) = lane3_simulation(1:3,car_count);
                %Removing the lane shifting node from lane3
                temp1 = [lane3_simulation(1,1:car_count-1) lane3_simulation(1,car_count+1:max_len_lane-1)];
                lane3_simulation(1,1:length(temp1)) = temp1;
                temp1 = [lane3_simulation(2,1:car_count-1) lane3_simulation(2,car_count+1:max_len_lane-1)];
                lane3_simulation(2,1:length(temp1)) = temp1;
                temp1 = [lane3_simulation(3,1:car_count-1) lane3_simulation(3,car_count+1:max_len_lane-1)];
                lane3_simulation(3,1:length(temp1)) = temp1;
                
            end
        end
    end
end

%lane changing from Lane4 to lane 3
for car_count = 1:num_cars_per_lane
    [value,index] = min(lane3_simulation(2,find(lane3_simulation(2,:) >lane4_simulation(2,car_count)))) ;
    empty_slot_ahead = value-10;
    c = max(lane3_simulation(2,find(lane3_simulation(2,:) < lane4_simulation(2,car_count))));
    empty_slot_behind = c + 10;
    if ~isempty(empty_slot_ahead) && ~isempty(empty_slot_behind)
        if empty_slot_ahead > (lane4_simulation(2,car_count) + saftey_dist) && empty_slot_behind<  (lane4_simulation(2,car_count)-saftey_dist)
            temp = [];
            temp1 = [];
            temp = [lane3_simulation(1,1:index) 0 lane3_simulation(1,index+1:max_len_lane-1)];
            lane3_simulation(1,1:length(temp)) = temp;
            temp = [lane3_simulation(2,1:index) 0 lane3_simulation(2,index+1:max_len_lane-1)];
            lane3_simulation(2,1:length(temp)) = temp;
            temp = [lane3_simulation(3,1:index) 0 lane3_simulation(3,index+1:max_len_lane-1)];
            lane3_simulation(3,1:length(temp)) = temp;
            lane3_simulation(1:3,index+1) = lane4_simulation(1:3,car_count);
            %Removing the lane shifting node from lane4
            temp1 = [lane4_simulation(1,1:car_count-1) lane4_simulation(1,car_count+1:max_len_lane-1)];
            lane4_simulation(1,1:length(temp1)) = temp1;
            temp1 = [lane4_simulation(2,1:car_count-1) lane4_simulation(2,car_count+1:max_len_lane-1)];
            lane4_simulation(2,1:length(temp1)) = temp1;
            temp1 = [lane4_simulation(3,1:car_count-1) lane4_simulation(3,car_count+1:max_len_lane-1)];
            lane4_simulation(3,1:length(temp1)) = temp1;
            
        end
    end
end

end