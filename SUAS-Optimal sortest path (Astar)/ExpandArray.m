function Expand_arr=ExpandArray(Node_x,Node_y,Hn,Target_x,Target_y,Closed_list,X_Grid_size,Y_Grid_size)
    Expand_arr=[];
    Expand_count=1;
    Closed_elements=size(Closed_list,1);
    
    for z= 1:-1:-1
         for p= 1:-1:-1
            if (z~=p || z~=0) 
                succ_x = Node_x+z;
                succ_y = Node_y+p;
                if( (succ_x >0 && succ_x <=X_Grid_size) && (succ_y >0 && succ_y <=Y_Grid_size))
                    temp=1;                    
                    for c=1:Closed_elements
                        if(succ_x == Closed_list(c,1) && succ_y == Closed_list(c,2))
                            temp=0;
                        end
                    end
                    if (temp == 1)
                        Expand_arr(Expand_count,1) = succ_x;
                        Expand_arr(Expand_count,2) = succ_y;
                        Expand_arr(Expand_count,3) = Hn+sqrt((Node_x-succ_x)^2 + (Node_y-succ_y)^2);
                        Expand_arr(Expand_count,4) = sqrt((succ_x-Target_x)^2 + (succ_y-Target_y)^2);
                        Expand_arr(Expand_count,5) = Expand_arr(Expand_count,3)+Expand_arr(Expand_count,4);
                        Expand_count=Expand_count+1;
                    end
                end
            end
        end
    end