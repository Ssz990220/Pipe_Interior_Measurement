classdef UR10StateSpaceGMM < nav.StateSpace & ...
        matlabshared.planning.internal.EnforceScalarHandle
    %UR10STATESPACE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        UniformDistribution
        NormalDistribution
    end
    
    methods
        function obj = UR10StateSpaceGMM
            spaceName = "UR10StateSpace";
            numStateVariables = 6;
            stateBounds = [ones(6,1)*-1*pi,ones(6,1)*pi];
            
            obj@nav.StateSpace(spaceName, numStateVariables, stateBounds);
            
            obj.NormalDistribution = matlabshared.tracking.internal.NormalDistribution(numStateVariables);
            obj.UniformDistribution = matlabshared.tracking.internal.UniformDistribution(numStateVariables);
            % User-defined property values here
        end
        
        function copyObj = copy(obj)
            printf("Copy Object");
        end
        
        function boundedState = enforceStateBounds(obj, state)
            nav.internal.validation.validateStateMatrix(state, nan, obj.NumStateVariables, "enforceStateBounds", "state");
            boundedState = state;
            boundedState = min(max(boundedState, obj.StateBounds(:,1)'), ...
                obj.StateBounds(:,2)');
        end
        
        function state = sampleUniform(obj, varargin)
            narginchk(1,4);
            [numSamples, stateBounds] = obj.validateSampleUniformInput(varargin{:});
            
            obj.UniformDistribution.RandomVariableLimits = stateBounds;
            state = obj.UniformDistribution.sample(numSamples);
        end
        
        function state = sampleGaussian(obj, meanState, stdDev, varargin)    
            narginchk(3,4);
            
            [meanState, stdDev, numSamples] = obj.validateSampleGaussianInput(meanState, stdDev, varargin{:});
            
            obj.NormalDistribution.Mean = meanState;
            obj.NormalDistribution.Covariance = diag(stdDev.^2);
            
            state = obj.NormalDistribution.sample(numSamples);
            state = obj.enforceStateBounds(state);
            
        end
        
        function state = sampleGMM(obj, GMM_model)
            state = random(GMM_model,1);
            state = cvt_n_space(state);
            state = obj.enforceStateBounds(state);
        end
        
        function interpState = interpolate(obj, state1, state2, fraction)
            narginchk(4,4);
            [state1, state2, fraction] = obj.validateInterpolateInput(state1, state2, fraction);
            
            stateDiff = state2 - state1;
            interpState = state1 + fraction' * stateDiff;
        end
        
        function states = sample_around_traj(obj,state1,state2,var,n_sample_per_state)
           dim = size(state1,2);
           interp_states = obj.interpolate(state1, state2, [0:(1/8):1 1]);
           states = [];
           for i = 1:size(interp_states,1)
               states = [states; obj.sampleGaussian(interp_states(i,:),ones(1,dim)*var,n_sample_per_state)];
           end
        end
        
        function new_samples = sample_around_states(obj, states, var, n_sample_per_state)
            new_samples = zeros(n_sample_per_state,size(states,2),size(states,1));
            for i = 1:size(states,1)
                new_samples(:,:,i) = obj.sampleGaussian(states(i,:),ones(size(states,2),1)*var,n_sample_per_state);
           end
        end
        
        function dist = distance(obj, state1, state2)
            
            narginchk(3,3);
            s1 = size(state1,1);
            s2 = size(state2,1);
            if s1 == s2
                nav.internal.validation.validateStateMatrix(state1, nan, obj.NumStateVariables, "distance", "state1");
                nav.internal.validation.validateStateMatrix(state2, size(state1,1), obj.NumStateVariables, "distance", "state2");

                stateDiff = state2 - state1;
                dist = sqrt( sum( stateDiff.^2, 2 ) );
            else
                if s1 == 1
                    stateDiff = repmat(state1,size(state2,1),1)-state2;
                    dist = sqrt( sum( stateDiff.^2, 2 ) );
                elseif s2 == 1
                    stateDiff = repmat(state2,size(state1,1),1)-state1;
                    dist = sqrt( sum( stateDiff.^2, 2 ) );
                else
                    nav.internal.validation.validateStateMatrix(state1, nan, obj.NumStateVariables, "distance", "state1");
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), obj.NumStateVariables, "distance", "state2");
                end
            end
        end
        
        
    end
end

