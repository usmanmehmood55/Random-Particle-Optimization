function AP_index = AP_selector(NPTS, error_d, error_j)

AP_index = 1;
S_error_d = error_d(1);
S_error_j = error_j(1);
for i = 1:NPTS
    
    if ((error_j(i) < 0) && (error_d(i) < 0))
        
        if (error_j(i) <= S_error_j)
            
            S_error_j = error_j(i);
            AP_index = i;
        end
    end
end
if AP_index == 1
    
    S_error_d = error_d(1);
    S_error_j = error_j(1);
    
    for i = 1:NPTS
        
        if error_d(i) <= S_error_d
            S_error_d = error_d(i);
            AP_index = i;
        end
    end
end

end