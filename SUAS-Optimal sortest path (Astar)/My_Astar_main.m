X_Grid_Size=10;
Y_Grid_Size=10;
Total_Value=10;
Terrain_Map=2*(ones(X_Grid_Size,Y_Grid_Size));
Rob_x_corr = 1; 
Rob_y_corr = 1;  
axis([1 X_Grid_Size+1 1 Y_Grid_Size+1])
grid on;    hold on;
Obs_no=0;

pause(1);
h=msgbox('Choose the target location using left mouse key');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
but=0;
while (but ~= 1) 
    [Rob_x_corr,Rob_y_corr,but]=ginput(1);
end
Rob_x_corr=floor(Rob_x_corr);
Rob_y_corr=floor(Rob_y_corr);
Target_x=Rob_x_corr;
Target_y=Rob_y_corr;

Terrain_Map(Rob_x_corr,Rob_y_corr)=0;
plot(Rob_x_corr+.5,Rob_y_corr+.5,'gd');
text(Rob_x_corr+1,Rob_y_corr+.5,'Target')

pause(2);
h=msgbox('choose the location of the obstacles using left mouse key,to complete the selection press the right mouse key');
 uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [Rob_x_corr,Rob_y_corr,but] = ginput(1);
    Rob_x_corr=floor(Rob_x_corr);
    Rob_y_corr=floor(Rob_y_corr);
    Terrain_Map(Rob_x_corr,Rob_y_corr)=-1;
    plot(Rob_x_corr+.5,Rob_y_corr+.5,'ro');
end
 
pause(1);

h=msgbox('choose the initial position of robot using left Mouse key');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end

but=0;
while (but ~= 1)
    [Rob_x_corr,Rob_y_corr,but]=ginput(1);
    Rob_x_corr=floor(Rob_x_corr);
    Rob_y_corr=floor(Rob_y_corr);
end
begin_x=Rob_x_corr;
begin_y=Rob_y_corr;
Terrain_Map(Rob_x_corr,Rob_y_corr)=1;
 plot(Rob_x_corr+.5,Rob_y_corr+.5,'bo');
Open_list=[];
Closed_list=[];

d_counter=1;
for x=1:X_Grid_Size
    for y=1:Y_Grid_Size
        if(Terrain_Map(x,y) == -1)
            Closed_list(d_counter,1)=x; 
            Closed_list(d_counter,2)=y; 
            d_counter=d_counter+1;
        end
    end
end
Closed_list_Count=size(Closed_list,1);
Node_x=Rob_x_corr;
Node_y=Rob_y_corr;
Open_list_count=1;
costofpath_cost=0;
distanceofGoal=sqrt((Node_x-Target_x)^2 + (Node_y-Target_y)^2);
Open_list(Open_list_count,:)=O_L_insertion(Node_x,Node_y,Node_x,Node_y,costofpath_cost,distanceofGoal,distanceofGoal);
Open_list(Open_list_count,1)=0;
Closed_list_Count=Closed_list_Count+1;
Closed_list(Closed_list_Count,1)=Node_x;
Closed_list(Closed_list_Count,2)=Node_y;
No_Path_find=1;

while((Node_x ~= Target_x || Node_y ~= Target_y) && No_Path_find == 1)
 Expand_arr=ExpandArray(Node_x,Node_y,costofpath_cost,Target_x,Target_y,Closed_list,X_Grid_Size,Y_Grid_Size);
 Expand_count=size(Expand_arr,1);
 
 for x=1:Expand_count
    flag=0;
    for y=1:Open_list_count
        if(Expand_arr(x,1) == Open_list(y,2) && Expand_arr(x,2) == Open_list(y,3) )
            Open_list(y,8)=min(Open_list(y,8),Expand_arr(x,5)); 
            if Open_list(y,8)== Expand_arr(x,5)
                Open_list(y,4)=Node_x;
                Open_list(y,5)=Node_y;
                Open_list(y,6)=Expand_arr(x,3);
                Open_list(y,7)=Expand_arr(x,4);
            end
            flag=1;
        end
    end
    if flag == 0
        Open_list_count = Open_list_count+1;
        Open_list(Open_list_count,:)=O_L_insertion(Expand_arr(x,1),Expand_arr(x,2),Node_x,Node_y,Expand_arr(x,3),Expand_arr(x,4),Expand_arr(x,5));
    end
 end
   Min_index_node = compute_min(Open_list,Open_list_count,Target_x,Target_y);
  if (Min_index_node ~= -1)    
    Node_x=Open_list(Min_index_node,2);
   Node_y=Open_list(Min_index_node,3);
   costofpath_cost=Open_list(Min_index_node,6);
  Closed_list_Count=Closed_list_Count+1;
  Closed_list(Closed_list_Count,1)=Node_x;
  Closed_list(Closed_list_Count,2)=Node_y;
  Open_list(Min_index_node,1)=0;
  else
      No_Path_find=0;
  end
end
x=size(Closed_list,1);
Opt_path=[];
Rob_x_corr=Closed_list(x,1);
Rob_y_corr=Closed_list(x,2);
x=1;
Opt_path(x,1)=Rob_x_corr;
Opt_path(x,2)=Rob_y_corr;
x=x+1;

if ( (Rob_x_corr == Target_x) && (Rob_y_corr == Target_y))
    ith_node=0;
   parentNode_x=Open_list(index_locate(Open_list,Rob_x_corr,Rob_y_corr),4);
   parentNode_y=Open_list(index_locate(Open_list,Rob_x_corr,Rob_y_corr),5);
   
   while( parentNode_x ~= begin_x || parentNode_y ~= begin_y)
           Opt_path(x,1) = parentNode_x;
           Opt_path(x,2) = parentNode_y;
           ith_node=index_locate(Open_list,parentNode_x,parentNode_y);
           parentNode_x=Open_list(ith_node,4);
           parentNode_y=Open_list(ith_node,5);
           x=x+1;
    end
 y=size(Opt_path,1);
 plot_p=plot(Opt_path(y,1)+.5,Opt_path(y,2)+.5,'bo');
 y=y-1;
 for x=y:-1:1
  pause(.25);
  set(plot_p,'XData',Opt_path(x,1)+.5,'YData',Opt_path(x,2)+.5);
 drawnow ;
 end
 plot(Opt_path(:,1)+.5,Opt_path(:,2)+.5);
else
 pause(1);
 h=msgbox('No path exists to the Target!','warn');
 uiwait(h,5);
end

