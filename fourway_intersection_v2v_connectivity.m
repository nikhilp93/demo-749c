

close all;

%Define a region of 1000X1000m and take input points for drawing two raods
axis([0 1000 0 1000]);
title('Four way intersection - V2V Connectivity Demonstration');
%taking ginput for first road
[r1_x,r1_y] = ginput(2);
line(r1_x,r1_y,'color','k');
hold on

%taking ginput for first road
[r2_x,r2_y] = ginput(2);
line(r2_x,r2_y,'color','k');

%Generating linearly spaced points on the two roads. The number of points
%will be a random number between 10 and 20.Starting point and intersection
%point
%Road1
line1 = randi([10,20]);
road1_x = linspace(r1_x(1),r1_x(2),line1);
road1_y = linspace(r1_y(1),r1_y(2),line1);

%Road2
line2 = randi([10,20]);
road2_x = linspace(r2_x(1),r2_x(2),line2);
road2_y = linspace(r2_y(1),r2_y(2),line2);

%%Find the intersection point between two roads--just for visualizing
pf1 = polyfit(r1_x,r1_y,1);
pf2 = polyfit(r2_x,r2_y,1);
xintersect = fzero(@(m) polyval(pf1 -pf2,m),3);
yintersect = polyval(pf1,xintersect);
intersection_pt = [xintersect,yintersect];
plot(xintersect,yintersect,'r*');



loop_count = 80;
%Number of iterations the nodes will move inside the region
for iter_count = 1:loop_count
    %Indices for updating location of the node for every iteration
    %interval.
    index_x1 = (mod(iter_count,line1)+1);
    index_y1 = (mod(iter_count,line1)+1);
    index_x2 = (mod(iter_count,line2)+1);
    index_y2 = (mod(iter_count,line2)+1);
    index_x3 = index_x2;
    index_y3 = index_y2;
    
    %handles for plotting the three nodes on the roads.
    handle1 = plot(road1_x(index_x1),road1_y(index_y1),':rd','MarkerFaceColor','r','MarkerSize',10);
    handle2 = plot(road2_x(index_x2),road2_y(index_y2),':gd','MarkerFaceColor','g','MarkerSize',10);
    handle3 = plot(road2_x(line2+1-index_x3),road2_y(line2+1-index_y3),':bd','MarkerFaceColor','b','MarkerSize',10);
    
    
    %calculating distances between the nodes.
    dist_12=sqrt(power(road2_x(index_x2)-road1_x(index_x1),2)+power(road2_y(index_y2)-road1_y(index_y1),2));
    dist_13=sqrt(power(road2_x(line2+1-index_x3)-road1_x(index_x1),2)+power(road2_y(line2+1-index_y3)-road1_y(index_y1),2));
    dist_23=sqrt(power(road2_x(line2+1-index_x3)-road2_x(index_x2),2)+power(road2_y(line2+1-index_y3)-road2_y(index_y2),2));
    
    %Ensuring that distances are less than or equal to 100 and plotting the
    %V2V connectivity.
    if(dist_12<=100||dist_13<=100||dist_23<=100)
        
        if dist_12<=100 && dist_23<=100 && dist_13<=100
            handle4 = plot([road1_x(index_x1) road2_x(index_x2)],[road1_y(index_y1) road2_y(index_y2)],'--k','LineWidth',3,'MarkerSize',20);
            handle5 = plot([road1_x(index_x1) road2_x(line2+1-index_x3)],[road1_y(index_y1) road2_y(line2+1-index_y3)],'--k','LineWidth',3,'MarkerSize',20);
            handle6 = plot([road2_x(line2+1-index_x3) road2_x(index_x2)],[road2_y(line2+1-index_y3) road2_y(index_y2)],'--k','LineWidth',3,'MarkerSize',20);
            pause(0.8);
            set(handle1,'Visible','off')
            set(handle2,'Visible','off')
            set(handle3,'Visible','off')
            set(handle4,'Visible','off')
            set(handle5,'Visible','off')
            set(handle6,'Visible','off')
        
        
        elseif dist_12<=100
            handle4 = plot([road1_x(index_x1) road2_x(index_x2)],[road1_y(index_y1) road2_y(index_y2)],'--k','LineWidth',3,'MarkerSize',20);
            pause(0.2);
            set(handle1,'Visible','off')
            set(handle2,'Visible','off')
            set(handle3,'Visible','off')
            set(handle4,'Visible','off')
        
        
        elseif dist_13 <=100
            handle5 = plot([road1_x(index_x1) road2_x(line2+1-index_x3)],[road1_y(index_y1) road2_y(line2+1-index_y3)],'--k','LineWidth',3,'MarkerSize',20);
            pause(0.2);
            set(handle1,'Visible','off')
            set(handle2,'Visible','off')
            set(handle3,'Visible','off')
            set(handle5,'Visible','off')
        
        elseif dist_23<=100
            handle6 = plot([road2_x(line2+1-index_x3) road2_x(index_x2)],[road2_y(line2+1-index_y3) road2_y(index_y2)],'--k','LineWidth',3,'MarkerSize',20);
            pause(0.2);
            set(handle1,'Visible','off')
            set(handle2,'Visible','off')
            set(handle3,'Visible','off')
            set(handle6,'Visible','off')
        end
        
    else
        pause(0.5)
        set(handle1,'Visible','off')
        set(handle2,'Visible','off')
        set(handle3,'Visible','off')
    end  
end
