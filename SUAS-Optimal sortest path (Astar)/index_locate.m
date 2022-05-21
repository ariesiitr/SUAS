function location_index = index_locate(Open_list,Rob_x_corr,Rob_y_corr)
    p=1;
    while(Open_list(p,2) ~= Rob_x_corr || Open_list(p,3) ~= Rob_y_corr )
        p=p+1;
    end
    location_index=p;
end