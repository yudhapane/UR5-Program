global URARM_INIT URARM_HANDLE

addpath(genpath(cd)); % add subdirectories

if isempty(URARM_INIT)
    URARM_INIT = false;
end

if ~URARM_INIT
    URARM_INIT = true;
    ip = '192.168.1.50';
    URARM_HANDLE = URArm;
    if ~URARM_HANDLE.isConnected()
        URARM_HANDLE.fopen(ip);
    end
end
