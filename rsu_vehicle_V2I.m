%Simulating V2I Coverage for a Vehicle in a Road Map with a RSU Deployment

close all;
%Define a region of 1000X1000m
axis([0 1000 0 1000]);
grid on
hold on

%Take input points for drawing 5 nodes and connecting roads.
nodes = input('Please enter the number of Nodes: ');

[node_x,node_y] = ginput(nodes);
plot(node_x,node_y,'k','LineWidth',2,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor','b',...
    'MarkerSize',4)
title('RSU to Vehicle Connectivity Demonstration');

%Take the coordinates of 3 RSu's as input
rsu = input('Please enter the number of RSUs (1/2/3): ');
[rsu_x,rsu_y] = ginput(rsu);

%Plot the RSU's
if(rsu==1)
    plot(rsu_x(1),rsu_y(1),'ob','MarkerFaceColor','b');
elseif(rsu==2)
    plot(rsu_x(1),rsu_y(1),'ob','MarkerFaceColor','b');
    plot(rsu_x(2),rsu_y(2),'ob','MarkerFaceColor','b');
elseif(rsu==3)
    plot(rsu_x(1),rsu_y(1),'ob','MarkerFaceColor','b');
    plot(rsu_x(2),rsu_y(2),'ob','MarkerFaceColor','b');
    plot(rsu_x(3),rsu_y(3),'ob','MarkerFaceColor','b');
end

%initializing the location of the first line to the first index.
rear_index_x = node_x(1);
rear_index_y = node_y(1);
speed = 0;
%loop_count=1;

% for loop = 1:loop_count
for i = 2:nodes
    front_index_x = node_x(i);
    front_index_y = node_y(i);
    rear_index_x = node_x(i-1);
    rear_index_y = node_y(i-1);
    speed = randi([10,20]);
    
    %Generating linearly spaced vectors which is analogous to the
    %vehicle movement.
    path_x = linspace(rear_index_x,front_index_x,speed);
    path_y = linspace(rear_index_y,front_index_y,speed);
    %We are interating over paths at the rate determined by the vehicle
    %speed.
    for path_iter = 2:speed
        %Set a handle for the vehicle and set it to display a yellow
        %square.
        handle_vehicle = plot(path_x(path_iter),path_y(path_iter),'sk','MarkerFaceColor','y','MarkerSize',10);
        pause(0.2);
        
        %Handle for plotting signal connectivity near rsu
        handle1 = plot(path_x(path_iter),path_y(path_iter),'sg','MarkerFaceColor','g','MarkerSize',8);
        
        %Calculate eucledian distances between the points in path and
        %the RSU
        dist_1 = Inf;
        dist_2 = Inf;
        dist_3 = Inf;
        
        if rsu==1
            dist_1=sqrt(power(path_x(path_iter)-rsu_x(1),2)+power(path_y(path_iter)-rsu_y(1),2));
        elseif rsu==2
            dist_1=sqrt(power(path_x(path_iter)-rsu_x(1),2)+power(path_y(path_iter)-rsu_y(1),2));
            dist_2=sqrt(power(path_x(path_iter)-rsu_x(2),2)+power(path_y(path_iter)-rsu_y(2),2));
        elseif rsu==3
            dist_1=sqrt(power(path_x(path_iter)-rsu_x(1),2)+power(path_y(path_iter)-rsu_y(1),2));
            dist_2=sqrt(power(path_x(path_iter)-rsu_x(2),2)+power(path_y(path_iter)-rsu_y(2),2));
            dist_3=sqrt(power(path_x(path_iter)-rsu_x(3),2)+power(path_y(path_iter)-rsu_y(3),2));
        end
        
        %Checking the proximity to given RSU within 100m.
        if(dist_1<=100 && dist_1<dist_2)
            handle2 = plot([path_x(path_iter) rsu_x(1)],[path_y(path_iter) rsu_y(1)]);
            pause(0.2);
            set(handle2,'Visible','off');
            continue
            
        elseif(dist_1>=100 && dist_2<=100 && dist_2<dist_3)
            handle3 = plot([path_x(path_iter) rsu_x(2)],[path_y(path_iter) rsu_y(2)]);
            pause(0.2);
            set(handle3,'Visible','off');
            continue
        elseif(dist_3<=100)
            handle4 = plot([path_x(path_iter) rsu_x(3)],[path_y(path_iter) rsu_y(3)]);
            pause(0.2);
            set(handle4,'Visible','off');
            continue;
        elseif(dist_1>100&&dist_2>100&&dist_3>100)
            handle5 = plot([path_x(path_iter) node_x(nodes)],[path_y(path_iter) node_y(nodes)],'sr','MarkerFaceColor','r','MarkerSize',8);
            pause(0.2);
            %set(h5,'Visible','off');
            continue
            
        end
        set(handle1,'Visible','off');
        set(handle5,'Visible','off');
        
    end
    set(handle_vehicle,'Visible','off');
    
end
% end


