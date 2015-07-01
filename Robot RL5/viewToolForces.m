figure;
ts = 1/125;
duration = 10;
N = 5/ts;
i = 1;
forces = zeros(6,1);
while(1)
    while(i<N)
        clf;
        tic;
        arm.update();
        forces(:,i) = arm.getToolForces();
        plot(forces(1,:)); hold on; plot(forces(2,:));
        plot(forces(3,:)); hold on; plot(forces(4,:));
        plot(forces(5,:)); hold on; plot(forces(6,:));
        grid on;
        xlim([1 N]);
        while toc <ts
        end
        pause(0.001);
        i = i+1;
    end
    i = 1;
    clear forces;
    forces = zeros(6,1);
end