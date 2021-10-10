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
            copyObj = feval(class(obj));
            copyObj.StateBounds = obj.StateBounds;
            copyObj.UniformDistribution = obj.UniformDistribution.copy;
            copyObj.NormalDistribution = obj.NormalDistribution.copy;
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
            state = GMM_model.sample();
            state = obj.enforceStateBounds(state);
        end
        
        function interpState = interpolate(obj, state1, state2, fraction)
            narginchk(4,4);
            [state1, state2, fraction] = obj.validateInterpolateInput(state1, state2, fraction);
            
            stateDiff = state2 - state1;
            interpState = state1 + fraction' * stateDiff;
        end
        
        function dist = distance(obj, state1, state2)
            
            narginchk(3,3);
            
            nav.internal.validation.validateStateMatrix(state1, nan, obj.NumStateVariables, "distance", "state1");
            nav.internal.validation.validateStateMatrix(state2, size(state1,1), obj.NumStateVariables, "distance", "state2");

            stateDiff = bsxfun(@minus, state2, state1);
            dist = sqrt( sum( stateDiff.^2, 2 ) );
        end
        
        
    end
end

