function [wrTraj, wrdTraj, time, N] = genTraj(opt, par, arm)
    EXPERIMENT_TIME = 10;
    SAMPLING_TIME   = par.ts;
    if opt == 1
        N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];
        arm.update();
        w0              = arm.getToolPositions();   % initial cartesian pose

        wrTraj     	= zeros(6,N);   % reference trajectory, will be populated in the iteration
        offset    	= [-0.25 0 0 0 0 0]';
        dOffset    	= offset/N;

        for i = 1:N
            wrTraj(:,i)	= w0 + (i-1)*dOffset; % populate reference trajectory
        end
         wrdTraj            = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
         wrdTraj(:,end+1)   = zeros(3,1);
    elseif opt == 2  % trapezoidal reference
        n1              = 3/SAMPLING_TIME;
        n2              = 7/SAMPLING_TIME;
        n3              = n1;
        N               = n1+n2+n3;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.10, n1);
        wrTraj(3,1:n1)  = linspace(w0(3), w0(3)+0.01, n1);
        wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.05, n2);
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.10, n3);
        wrTraj(3,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(3,n1+n2), wrTraj(3,n1+n2)-0.01, n3);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
	elseif opt == 3 % ramp, ascending
        N               = 3/SAMPLING_TIME;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:N)  = repmat(w0,[1,N]);
        wrTraj(1,1:N)  = linspace(w0(1), w0(1)-0.05, N);
        wrTraj(3,1:N)  = linspace(w0(3), w0(3)+0.01, N);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);	
	elseif opt == 4 % ramp, descending
        N               = 3/SAMPLING_TIME;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:N)  = repmat(w0,[1,N]);
        wrTraj(1,1:N)  = linspace(w0(1), w0(1)-0.10, N);
        wrTraj(3,1:N)  = linspace(w0(3), w0(3)-0.005, N);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
    elseif opt == 5 % square reference (used for training)
        n1              = 2/SAMPLING_TIME;
        n2              = 5/SAMPLING_TIME;
        n3              = 4/SAMPLING_TIME;
        N               = n1+n2+n3;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.03, n1);
        wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.1, n2);
        wrTraj(3,n1+1:n1+n2)    = wrTraj(3,n1)+0.005;
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.1, n3);
        wrTraj(3,n1+n2+1:n1+n2+n3)    = wrTraj(3,n1);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
    elseif opt == 6 % step at t = 0
        N               = 10/SAMPLING_TIME;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:N)  = repmat(w0,[1,N]);
        wrTraj(1,1:N)  = linspace(w0(1), w0(1)-0.15, N);
        wrTraj(3,1:N)  = wrTraj(3,1)+0.005;

        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);      
    elseif opt == 7 % test reference, square varying amplitude
        n1              = 3/SAMPLING_TIME;
        n2              = 4/SAMPLING_TIME;
        n3              = 4/SAMPLING_TIME;
        n4              = 4/SAMPLING_TIME;
        N               = n1+n2+n3+n4;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();
        
        % 1-st segment
        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.03, n1);
        
        % 2-nd segment
        wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.07, n2);
        wrTraj(3,n1+1:n1+n2)    = wrTraj(3,n1)+0.003;
        
        % 3-rd segment
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.07, n3);
        wrTraj(3,n1+n2+1:n1+n2+n3)    = wrTraj(3,n1+n2)+0.002;

        % 4-th segment
        wrTraj(:,n1+n2+n3+1:N)    = repmat(wrTraj(:,n1+n2+n3),[1,n4]);
        wrTraj(1,n1+n2+n3+1:N)    = linspace(wrTraj(1,n1+n2+n3), wrTraj(1,n1+n2+n3)-0.05, n4);
        wrTraj(3,n1+n2+n3+1:N)    = wrTraj(3,n1+n2+n3)-0.004;
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
    elseif opt == 8 % test reference, square varying amplitude 2
        n1              = 3/SAMPLING_TIME;
        n2              = 4/SAMPLING_TIME;
        n3              = 4/SAMPLING_TIME;
        n4              = 4/SAMPLING_TIME;
        n5              = 4/SAMPLING_TIME;
        N               = n1+n2+n3+n4+n5;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();
        
        % 1-st segment
        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.03, n1);
        
        % 2-nd segment
        wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.07, n2);
        wrTraj(3,n1+1:n1+n2)    = wrTraj(3,n1)+0.003;
        
        % 3-rd segment
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.07, n3);
        wrTraj(3,n1+n2+1:n1+n2+n3)    = wrTraj(3,n1+n2)+0.002;

        % 4-th segment
        wrTraj(:,n1+n2+n3+1:N-n5)    = repmat(wrTraj(:,n1+n2+n3),[1,n4]);
        wrTraj(1,n1+n2+n3+1:N-n5)    = linspace(wrTraj(1,n1+n2+n3), wrTraj(1,n1+n2+n3)-0.05, n4);
        wrTraj(3,n1+n2+n3+1:N-n5)    = wrTraj(3,n1+n2+n3)-0.004;
        
        % 5-th segment
        wrTraj(:,n1+n2+n3+n4+1:N)   = repmat(wrTraj(:,n1+n2+n3+n4),[1,n5]);
        wrTraj(1,n1+n2+n3+n4+1:N)   = linspace(wrTraj(1,n1+n2+n3+n4), wrTraj(1,n1+n2+n3+n4)-0.05, n5);
        wrTraj(3,n1+n2+n3+n4+1:N)   = wrTraj(3,n1+n2+n3+n4)+0.003;
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
    elseif opt == 9 % trapezoidal reference 2 (somehow results in weird tracking)
        n1              = 3/SAMPLING_TIME;
        n2              = 3/SAMPLING_TIME;
        n3              = 7/SAMPLING_TIME;
        n4              = 3/SAMPLING_TIME;
        n5              = 3/SAMPLING_TIME;
        N               = n1+n2+n3+n4+n5;
        
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();
        
        % 1-st segment
        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.10, n1);
        
        % 2-nd segment
        wrTraj(:,n1+1:n1+n2)  = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)  = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.10, n2);
        wrTraj(3,n1+1:n1+n2)  = linspace(wrTraj(3,n1), wrTraj(3,n1)+0.004, n2);

        % 3-rd segment
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1)-0.05, n3);

        % 4-th segment
        wrTraj(:,n1+n2+n3+1:n1+n2+n3+n4)    = repmat(wrTraj(:,n1+n2+n3),[1,n4]);
        wrTraj(1,n1+n2+n3+1:n1+n2+n3+n4)    = linspace(wrTraj(1,n1+n2+n3), wrTraj(1,n1+n2+n3)-0.10, n4);
        wrTraj(3,n1+n2+n3+1:n1+n2+n3+n4)    = linspace(wrTraj(3,n1+n2+n3), wrTraj(3,n1+n2+n3)-0.004, n4);
        
        % 5-th segment
        wrTraj(:,n1+n2+n3+n4+1:N)    = repmat(wrTraj(:,n1+n2+n3+n4),[1,n5]);
        wrTraj(1,n1+n2+n3+n4+1:N)    = linspace(wrTraj(1,n1+n2+n3+n4), wrTraj(1,n1+n2+n3+n4)-0.10, n5);
        
        
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);        
    end