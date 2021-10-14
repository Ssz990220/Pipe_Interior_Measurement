classdef planner_tester
    %PLANNER_TESTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        planner
    end
    
    methods
        function obj = planner_tester(planner)
            %PLANNER_TESTER Construct an instance of this class
            %   Detailed explanation goes here
            obj.planner = planner;
        end
        
        function init_tester(obj)
            col_result = obj.planner.StateValidator.isStateValid_GMM(obj.planner.init_samples...
                (obj.planner.init_samples_col_flags,:),obj.planner.gmm_col_model_final, ...
                obj.planner.gmm_free_model_final);
%             col_result(81:90)
            sum(~col_result)/size(obj.planner.init_samples(obj.planner.init_samples_col_flags,:),1)
            free_result = obj.planner.StateValidator.isStateValid_GMM(obj.planner.init_samples...
                (~obj.planner.init_samples_col_flags,:),obj.planner.gmm_col_model_final, ...
                obj.planner.gmm_free_model_final);
            sum(free_result)/size(obj.planner.init_samples(~obj.planner.init_samples_col_flags,:),1)
        end
    end
end

