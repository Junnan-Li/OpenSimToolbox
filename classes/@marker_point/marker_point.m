classdef marker_point < handle

    % Initialize simulation  
%     properties (Constant)      
%         stepsize = 0.01; 
%     end
    
    
    properties
        body_name
        attached_body_name
        p
        R
        
        
    end

    methods
        function mp = marker_point(body_name,attached_body_name, p, R)
            assert(length(p)==3,'plot_frame: incorrect input dimension!')
            assert(isequal(size(R),[3 3]),'plot_frame: incorrect input dimension!')
            mp.body_name = body_name;
            mp.attached_body_name = attached_body_name;
            mp.p = p;
            mp.R = R;

        end 
            
    end
end
