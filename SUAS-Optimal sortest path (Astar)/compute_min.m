function min_val = compute_min(Open_list,Open_list_count,Target_x,Target_y)

 Ref_array=[];z=1;counter=0; goal_index_node=0;
  for p=1:Open_list_count
     if (Open_list(p,1)==1)
         Ref_array(z,:)=[Open_list(p,:) p]; 
         if (Open_list(p,2)==Target_x && Open_list(p,3)==Target_y)
             counter=1;
             goal_index_node=p;
         end
         z=z+1;
     end
  end
 if counter == 1 
    min_val=goal_index_node;
 end
 if size(Ref_array ~= 0)
  [min_fn,count_min]=min(Ref_array(:,8));
  min_val=Ref_array(count_min,9);
 else
     min_val=-1;
 end