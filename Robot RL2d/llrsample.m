function a = llrsample
%SAMPLE sample from LLR indices
    llrindices = 1:1:625;
    pc = cumsum(llrindices);
    a = find(pc>pc(end)*rand,1);
end