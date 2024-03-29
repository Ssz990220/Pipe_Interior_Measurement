classdef GMM_K_means < handle
    %GLOBAL_K_MEANS Summary of this class goes here
    %   Detailed explanation goes here
    %   Likas A, Vlassis N, Verbeek J J. The global k-means clustering algorithm[J]. 
    %   Pattern recognition, 2003, 36(2): 451-461.
    %   Huh J, Lee D D. Learning high-dimensional mixture models for fast collision detection 
    %   in rapidly-exploring random trees[C]//2016 IEEE International Conference on 
    %   Robotics and Automation (ICRA). IEEE, 2016: 63-69.
    
    properties
        data                % n*dim_data matrix, each row as one data point
        n_data
        dim_data            % scalar, dimension of the input data
        centers             % cell like {1*dim_data},{2*dim_data}..., cluster centers
        k_idx               % matrix: n_data * n_clusters
        % K-means Properties
        dis_mat             % n*n
        N                   % Maximum number of clusters
        n_iter              % number of iter has been done
        n_list              % list of number of clusters have been found
        k_means_options     % options for k-means algorithm
        new_x_init          % new x added in k+1 cluster
        % GMM Properties
        gmm_models          % gmm models for n clusters
        gmm_CovarianceType  % gmm covariance type, 'full' for default.
        gmm_SharedCovariance
                            % gmm covariance type, False for default
        gmm_options         % gmm options
        gmm_final           % final gmm model
        % Algorithm Parameters
        max_iter            % maximum iterations
        bhat_dis_threshold  % Bhattacharyya distance threshold for merging different clusters
        start_merge_threshold
                            % Start merging close clusters after finding
                            % start_merge_threshold clusters
        stop_criteria       % stop the algorithm after number of clusters stay still 
                            % for stop_critera iters.
        % Verbose
        display             % plot step result for debugging
    end
    
    methods
        function obj = GMM_K_means(X,N,options)
            %GMM_K_means Construct an instance of this class
            %   X is the sampled points in state space (ss)
            %   N is the maximum number of clusters
            obj.max_iter = options.max_iter;
            obj.bhat_dis_threshold = options.bhat_dis_threshold;
            obj.start_merge_threshold = options.start_merge_threshold;
            obj.stop_criteria = options.stop_criteria;
            obj.data = X;
            obj.dim_data = size(X,2);
            obj.n_data = size(X,1);
            obj.N = N;
            %   Prepare distance matrix to accelerate further calculation
            dis_mat = zeros(obj.n_data,obj.n_data);
            obj.centers = cell(N,1);
            parfor i = 1:obj.n_data
               dis_mat(i,:)=sum((X(i,:)-X).^2,2);         % How to accelerate?
            end
            obj.dis_mat = dis_mat;
            obj.n_list = zeros(1,obj.max_iter);
            obj.n_iter = 1;
            obj.k_idx = zeros(obj.n_data,obj.N);
            [obj.k_idx(:,1),obj.centers{1}] = kmeans(obj.data, 1);
            obj.k_means_options = statset('Display','off');       %TODO
            
            % GMM 
            obj.gmm_options = statset('Display','off',"MaxIter",500); %TODO
            obj.gmm_CovarianceType = 'full';
            obj.gmm_SharedCovariance = false;
            S.mu = obj.centers{1};
            S.Sigma = eye(obj.dim_data);
            S.ComponentProportion = 1;
            obj.gmm_models{1} = fitgmdist(obj.data, 1, 'CovarianceType',...
                obj.gmm_CovarianceType,'SharedCovariance',...
                obj.gmm_SharedCovariance, 'Options',obj.gmm_options,...
                'Start',S);
            obj.n_list(1) = 1;
        end
        
        function obj = BUILD_GMM(obj)
            for i = 1:obj.max_iter
                obj = obj.next_iter();
                if obj.n_iter < obj.stop_criteria
                    if var(obj.n_list(1:obj.n_iter))==0
                        obj.gmm_final = obj.gmm_models{end};
                        break
                    end
                elseif var(obj.n_list((obj.n_iter-obj.stop_criteria+1):obj.n_iter))==0
                    obj.gmm_final = obj.gmm_models{obj.n_iter};
                    return
                end
            end
            obj.gmm_final = obj.gmm_models{obj.n_iter};
        end
        
        function in_collision = check_collisions(obj, poses)
            n_pose = size(poses,1);
            in_collision = boolean(zeros(n_pose,1));
            parfor i = 1:n_pose
                in_collision(i) = check_one_collision_gmm(obj, poses(i,:));
            end
        end
        
        function points = sample(obj, n_sample)
            if nargin == 0
                points = zeros(1, obj.dim_data);                %TODO
            else
                points = zeros(n_sample, obj.dim_data);         %TODO
            end
        end
    end
    
    methods(Access='protected')
        
        function x_kp1_idx = find_xk_plus_1(obj)
            %find_k_plus_1_init finds the next initial center for
            %clustering.
            b = zeros(obj.n_data,1);
            data_local = obj.data;
            par_centers = obj.centers{obj.n_iter};
            data_local = repmat(data_local,1,1,size(par_centers,1));
            data_local = permute(data_local,[3,2,1]);
            par_centers = repmat(par_centers,1,1,obj.n_data);
            dis_mat_local = obj.dis_mat;
            num_data = obj.n_data;
            parfor i = 1:num_data
                d_kj = min(sum((par_centers - data_local).^2, 2),[],1);
                d_kj = permute(d_kj,[3,1,2]);
                b(i) = sum(max([d_kj' - dis_mat_local(i,:);zeros(1,num_data)],[],1));
            end
            [~,x_kp1_idx]= max(b);
        end
        
        function obj = next_iter(obj)
            id = obj.n_iter + 1;
            new_idx = obj.find_xk_plus_1();
            obj.new_x_init = obj.data(new_idx,:);
            % EM algorithm
            S.mu = [obj.centers{obj.n_iter};obj.new_x_init];
            S.Sigma = cat(3,obj.gmm_models{obj.n_iter}.Sigma,eye(obj.dim_data));
            S.ComponentProportion = ones(1,length(obj.gmm_models{obj.n_iter}.ComponentProportion)+1)...
                /(length(obj.gmm_models{obj.n_iter}.ComponentProportion)+1);
%             obj.gmm_models{id} = fitgmdist(obj.data, size(S.mu,1), ...
%                 'CovarianceType',obj.gmm_CovarianceType,...
%                 'SharedCovariance',obj.gmm_SharedCovariance, ...
%                 'RegularizationValue',0.001,...
%                 'Options',obj.gmm_options,'Start',S);             
            obj.gmm_models{id} = fitgmdist(obj.data, size(S.mu,1), ...
                'CovarianceType',obj.gmm_CovarianceType,...
                'SharedCovariance',obj.gmm_SharedCovariance, ...
                'RegularizationValue',0.001,...
                'Options',obj.gmm_options);             % TODO
            if obj.n_list(obj.n_iter) >= obj.start_merge_threshold
                [merge_flag, cur_centers] = merge_close_clusters(obj,obj.gmm_models{id});
                if merge_flag
                    S.mu = cur_centers;
                    if size(cur_centers,1)<obj.n_list(obj.n_iter)
                        obj.gmm_models{id} = fitgmdist(obj.data, size(S.mu,1), ...
                                'CovarianceType',obj.gmm_CovarianceType,...
                                'SharedCovariance',obj.gmm_SharedCovariance, ...
                                'Options',obj.gmm_options);
                    else
                        S.Sigma = obj.gmm_models{obj.n_iter}.Sigma;
                        S.ComponentProportion = obj.gmm_models{obj.n_iter}.ComponentProportion;
                        obj.gmm_models{id} = fitgmdist(obj.data, size(S.mu,1), ...
                        'CovarianceType',obj.gmm_CovarianceType,...
                        'SharedCovariance',obj.gmm_SharedCovariance, ...
                        'RegularizationValue',0.001,...
                        'Options',obj.gmm_options);%,'Start',S);
                    end
                end
            else
                cur_centers =[obj.centers{obj.n_iter};obj.new_x_init];
            end
            [obj.k_idx(:,id), obj.centers{id}] =...
                kmeans(obj.data,[],'Options',obj.k_means_options,'Start',...
                cur_centers,'Display','off');
            
            % Update obj variables
            obj.n_iter = id;
            obj.n_list(obj.n_iter) = size(cur_centers,1);
            fprintf('Iteration %d is done, found %d clustters...\n',[id, size(obj.centers{id},1)]);
        end
        
        function [merge_flag, centers] = merge_close_clusters(obj, model)
            % Only merge the closest one
            dis = bhat_dis(model.mu, model.Sigma);
            [~, min_idx] = min(dis, [], 2);
            dis_flag = boolean(zeros(size(dis)));
            for i = 1:length(min_idx)
                dis_flag(i,min_idx(i))=true;
            end
            dis_flag = dis_flag & (dis < obj.bhat_dis_threshold);
            if ~any(dis_flag,'all')
                centers = model.mu;
                merge_flag = false;
                return
            end
            merge_flag = true;
            is_merged = boolean(zeros(1,length(min_idx)));
            idx = cluster(model, obj.data);
            counter = 1;
            for i = 1:size(model.mu,1)
                if is_merged(i)
                    continue
                end
                cluster_points = (idx==i);
                centers(counter,:)=model.mu(i,:);
                for j = (i+1):size(model.mu,1)
                    if dis_flag(i,j) & ~is_merged(j)
                        cluster_points = cluster_points | (idx == j);
                        centers(counter,:)=mean(obj.data(cluster_points,:));
                        is_merged(j)=true;
                        continue
                    end
                end
                counter = counter + 1;
            end      
        end
        
        function display_gmm(obj)
            
        end
        
        function in_collision = check_one_collision_gmm(obj, pose)
            mahal(obj.gm_final, pose) % TODO
            in_collision = true; 
        end
    end
end