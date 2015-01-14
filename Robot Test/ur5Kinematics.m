%UR5KINEMATICS  create a ur5 robot kinematic object 
% 
% This function makes use of matlab robotic toolbox to define the kinematics
% of UR5 robot and return the robot object
% 
% Reference:  This function used robotics toolbox (rvc toolbox) from 
% Peter I. Corke (http://www.petercorke.com/Robotics_Toolbox.html_)
% 
% Yudha Prawira Pane (c)
% Created on Jan-14-2015

function UR5 = ur5Kinematics()
    % Denavit-hartenberg parameters
    d =         [0.089159   0       0           0.10915 0.09465	0.0823  ];
    r =         [0          -0.425	-0.39225  	0     	0       0       ];
    alpha =     [pi/2       0       0           pi/2    -pi/2   0       ];

    for i = 1:6
        L(i) = Link([0 d(i) r(i) alpha(i)]);
    end

    L(1).qlim = pi/180*[-360 360];
    L(2).qlim = pi/180*[-360 360];
    L(3).qlim = pi/180*[-360 360];
    L(4).qlim = pi/180*[-360 360];
    L(5).qlim = pi/180*[-360 360];
    L(6).qlim = pi/180*[-360 360];

    UR5 = SerialLink(L); % connect the links in series
    UR5.name = 'UR5';    % name the robot

