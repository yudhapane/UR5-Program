function q = selectJointPos(qik, qact)
        qik = abs(qik'); qact = abs(qact);
        err = qik-repmat(qact,1,8);
        