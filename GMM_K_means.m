classdef GMM_K_means
    %GLOBAL_K_MEANS Summary of this class goes here
    %   Detailed explanation goes here
    %   Likas A, Vlassis N, Verbeek J J. The global k-means clustering algorithm[J]. 
    %   Pattern recognition, 2003, 36(2): 451-461.
    
    properties
        data                % n*dim_data matrix, each row as one data point
        n_data
        dim_data            % scalar, dimension of the input data
        centers             % cell like {1*dim_data},{2*dim_data}..., cluster centers
        k_idx               % matrix: n_data * n_clusters
        % K-means Properties
        dis_mat             % n*n
        N                   % Maximum number of clusters
        n_cluster           % number of clusters has been found
        k_means_ops         % options for k-means algorithm
        % GMM Properties
        Property2
    end
    
    methods
        function obj = GMM_K_means(X,N)
            %GMM_K_means Construct an instance of this class
            %   X is the sampled points in state space (ss)
            %   N is the maximum number of clusters
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
            obj.n_cluster = 1;
            obj.k_idx = zeros(obj.n_data,obj.N);
            [obj.k_idx(:,1),obj.centers{1}] = kmeans(obj.data, 1);
            obj.k_means_ops = [];       %TODO
        end
        
        function x_kp1_idx = find_xk_plus_1(obj)
            %find_k_plus_1_init finds the next initial center for
            %clustering.
            b = zeros(obj.n_data,1);
            for i = 1:obj.n_data
                for j = 1:obj.n_data
                    d_kj = min(sum((obj.centers{obj.n_cluster}-...
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
        
        function next_k_means(obj)
            obj.n_cluster = obj.n_cluster + 1;
            new_idx = find_xk_plus_1();
            new_x_init = obj.data(new_idx,:);
            [obj.k_idx(:,obj.n_cluster), obj.centers{obj.n_cluster+1}] =...
                kmeans(obj.data,[],'Options',obj.k_means_ops,'Start',...
                [obj.centers{obj.n_cluster - 1};new_x_init],'Display','final');
        end
        
        function points = GMM_sample(obj,inputArg)
            % todo
            points = [];
        end
    end
end

