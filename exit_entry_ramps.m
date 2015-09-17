%Function to implement exit and entry at the ramps on four lanes
function [lane_simulatiom] = exit_entry_ramps(lane_simulatiom,current_entry_ramp_pos,current_exit_ramp_pos,...
    safe_dist_entry_forward,safe_dist_entry_back,target_node,lane_indicator)

%Find a node closets to the exit ramp
exit_node_arr = find(lane_simulatiom(2,:) <= current_exit_ramp_pos);
exit_node = lane_simulatiom(3,exit_node_arr(1));

if lane_indicator==1
    if exit_node ~= target_node
        if current_entry_ramp_pos > current_exit_ramp_pos
            
            %Finding the position of the 1st node behind the entry ramp and
            %assigning that position to the entering node.
            node_behind_entry = find(lane_simulatiom(2,:)< safe_dist_entry_back);
            entry_node = node_behind_entry(1);
            
            %Shifting the nodes between the entry and exit ramp down by 1
            if entry_node - exit_node < 0
                lane_simulatiom(2,entry_node:exit_node-1) = lane_simulatiom(2,entry_node:exit_node-1)+1;
            end
            
            %Entry of the node in the lane
            lane_simulatiom(2,entry_node)=current_entry_ramp_pos;
            
        end
        %If entry ramp is behind the exit ramp
        if current_entry_ramp_pos < current_exit_ramp_pos
            
            %Finding the position of the 1st node behind the entry ramp and
            %assigning that position to the entering node.
            node_behind_entry = find(lane_simulatiom(2,:)< safe_dist_entry_back);
            entry_node = node_behind_entry(1)-1;
            
            %Shifting the nodes between the entry and exit ramp down by 1
            if entry_node - exit_node > 0
                lane_simulatiom(2,entry_node:exit_node+1) = lane_simulatiom(2,entry_node:exit_node+1)-1;
            end
            
            %Entry of the node in the lane
            lane_simulatiom(2,entry_node)=current_entry_ramp_pos;
            lane_simulatiom(1,entry_node)=randi([50,70],1)*0.4470;
            
            
        end
    end
else
    if current_entry_ramp_pos > current_exit_ramp_pos
        
        %Finding the position of the 1st node behind the entry ramp and
        %assigning that position to the entering node.
        node_behind_entry = find(lane_simulatiom(2,:)< safe_dist_entry_back);
        entry_node = node_behind_entry(1);
        
        %Shifting the nodes between the entry and exit ramp down by 1
        if entry_node - exit_node < 0
            lane_simulatiom(2,entry_node:exit_node-1) = lane_simulatiom(2,entry_node:exit_node-1)+1;
        end
        
        %Entry of the node in the lane
        lane_simulatiom(2,entry_node)=current_entry_ramp_pos;
        
    end
    %If entry ramp is behind the exit ramp
    if current_entry_ramp_pos < current_exit_ramp_pos
        
        %Finding the position of the 1st node behind the entry ramp and
        %assigning that position to the entering node.
        node_behind_entry = find(lane_simulatiom(2,:)< safe_dist_entry_back);
        entry_node = node_behind_entry(1)-1;
        
        %Shifting the nodes between the entry and exit ramp down by 1
        if entry_node - exit_node > 0
            lane_simulatiom(2,entry_node:exit_node+1) = lane_simulatiom(2,entry_node:exit_node+1)-1;
        end
        
        %Entry of the node in the lane
        lane_simulatiom(2,entry_node)=current_entry_ramp_pos;
        lane_simulatiom(1,entry_node)=randi([50,70],1)*0.4470;
    end
end
end