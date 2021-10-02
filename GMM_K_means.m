classdef GMM_K_means
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
    end
    
    methods
        function obj = GMM_K_means(X,N,options)
            %GMM_K_means Construct an instance of this class
            %   X is the sampled points in state space (ss)
            %   N is the maximum number of clusters
            obj.max_iter = options.max_iter;
            obj.bhat_dis_threshold = options.bhat_dis_threshold;
            obj.start_merge_threshold = start_merge_threshold;
            obj.stop_criteria = options.stop_criteria;
            tic
            parpool
            t = toc;
            fprintf('Took %.2f sec to create a parallel pool\n',t);
            obj.data = X;
            obj.dim_data = size(X,1);
            obj.n_data = size(X,2);
            obj.N = N;
            %   Prepare distance matrix to accelerate further calculation
            obj.dis_mat = zeros(obj.n_data,obj.n_data);
            obj.centers = cell(N,1);
            for i = 1:obj.n_data
                for j = i+1:obj.n_data
                   obj.dis_mat(i,j)=sum((obj.data(i,:)-obj.data(j,:)).^2);
                end
            end
            
            obj.n_list = zeros(1,obj.max_iter);
            obj.n_iter = 1;
            obj.k_idx = zeros(obj.n_data,obj.N);
            [obj.k_idx(:,1),obj.centers{1}] = kmeans(obj.data, 1);
            obj.k_means_options = [];       %TODO
            
            % GMM 
            obj.gmm_options = []; %TODO
            obj.gmm_CovarianceType = 'full';
            obj.gmm_SharedCovariance = False;
            S.mu = obj.centers{1};
            S.Sigma = eye(obj.dim_data);
            S.ComponentProportion = 1;
            obj.gmm_models(1) = fitgmmdist(obj.data, 1, 'CovarianceType',...
                obj.gmm_CovarianceType,'SharedCovariance',...
                obj.gmm_SharedCovariance, obj.gmm_options,...
                'Start',S);
            obj.n_list(1) = 1;
        end
        
        function obj = BUILD_GMM(obj)
            for i = 1:obj.max_iter
                next_iter();
                if var(obj.n_list(-obj.stop_criteria:-1))==0
                    obj.gmm_final = obj.gmm_models(-1);
                end
            end
        end
        
        function in_collision = check_collision(obj, poses)
            n_pose = size(poses,1);
            in_collision = boolean(zeros(n_pose,1));
            parfor i = 1:n_pose
                in_collision(i) = check_one_collision_gmm(obj, poses(i,:));
            end
        end
        
        function points = sample(obj, n_sample)
           points = zeros(n_sample, obj.dim_data);
        end
    end
    
    methods(Access='protected')
        
        function x_kp1_idx = find_xk_plus_1(obj)
            %find_k_plus_1_init finds the next initial center for
            %clustering.
            b = zeros(obj.n_data,1);
            for i = 1:obj.n_data
                for j = 1:obj.n_data
                    d_kj = min(sum((obj.centers{obj.n_iter}-...
                        obj.data(j,:)).^2, 2));
                    if i < j
                        b(i) = b(i) + max(d_kj - obj.dis_mat(i,j),0);
                    else 
                        b(i) = b(i) + max(d_kj - obj.dis_mat(j,i),0);
                    end
                end
            end
            [~,x_kp1_idx]= max(b);
        end
        
        function obj = next_k_means(obj)
            %
            % must execute before next_gmm
            obj.n_cluster = obj.n_cluster + 1;
            [obj.k_idx(:,obj.n_cluster), obj.centers{obj.n_cluster+1}] =...
                kmeans(obj.data,[],'Options',obj.k_means_options,'Start',...
                [obj.centers{obj.n_cluster - 1};obj.new_x_init],'Display','final');
            new_idx = find_xk_plus_1();
            obj.new_x_init = obj.data(new_idx,:);
        end
        
        function obj = next_gmm(obj)
            S.mu = [obj.centers{obj.n_cluster - 1};obj.new_x_init];
            S.sigma = [obj.gmm_models(obj.n_cluster - 1).Sigma;eye(obj.dim_data)];
            S.ComponentProportion = [obj.gmm_models(obj.n_cluster-1).ComponentProportion,0];
            obj.gmm_models(obj.n_cluster) = fitgmmdist(obj.data, 1, ...
                'CovarianceType',obj.gmm_CovarianceType,...
                'SharedCovariance',obj.gmm_SharedCovariance, ...
                obj.gmm_options,'Start',S);
        end
        
        function obj = next_iter(obj)
            new_idx = find_xk_plus_1();
            obj.new_x_init = obj.data(new_idx,:);
            id = obj.n_iter + 1;
            % EM algorithm
            S.mu = [obj.centers{obj.n_iter};obj.new_x_init];
            S.sigma = cat(3,obj.gmm_models(obj.n_iter).Sigma,eye(obj.dim_data));
            S.ComponentProportion = [obj.gmm_models(obj.n_iter).ComponentProportion,0];
            obj.gmm_models(id) = fitgmmdist(obj.data, 1, ...
                'CovarianceType',obj.gmm_CovarianceType,...
                'SharedCovariance',obj.gmm_SharedCovariance, ...
                obj.gmm_options,'Start',S);
            if obj.n_list(obj.n_iter) >= obj.start_merge_threshold
                cur_centers = merge_close_clusters(obj);
            else
                cur_centers = obj.centers(obj.n_iter);
            end
            [obj.k_idx(:,id), obj.centers{id}] =...
                kmeans(obj.data,[],'Options',obj.k_means_options,'Start',...
                cur_centers,'Display','final');
            
            % Update obj variables
            obj.n_iter = id;
            obj.n_list(obj.n_iter) = size(cur_centers,1);
        end
        
        function centers = merge_close_clusters(obj, model)
            % Only merge the closest one
            dis = bhat_dis(model.mu, model.Sigma);
            [~, min_idx] = min(dis, [], 2);
            dis_flag = boolen(zeros(size(dis)));
            for i = 1:length(min_idx)
                dis_flag(i,min_idx(i))=true;
            end
            dis_flag = dis_flag & (dis < obj.bhat_dis_threshold);
            is_merged = boolen(zeros(1,length(min_idx)));
            idx = cluster(model, obj.data);
            counter = 0;
            for i = 1:size(model.mu,2)
                if is_merged(i)
                    continue
                end
                cluster_points = (idx==i);
                centers(counter,:)=model.mu(i,:);
                for j = (i+1):size(model.mu,2)
                    if dis_flag(i,j)
                        cluster_points = cluster_points & (idx == j);
                        centers(counter,:)=mean(obj.data(cluster_points,:));
                        is_merged(j)=true;
                        counter = counter + 1;
                        continue
                    end
                end
            end      
        end
        
        function in_collision = check_one_collision_gmm(obj, pose)
            mahal(obj.gm_final, pose) % TODO
            in_collision = true; 
        end
    end
end

