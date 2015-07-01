plot(par.wrTraj2{80}(3,:))
hold on;
plot(par.wTable{80}(3,:))
plot(par.wnomplotTable{80}(3,:))
format long;
err2 = rms(par.wrTraj2{80}(3,:)-par.wnomplotTable{80}(3,:));
err1 = rms(par.wrTraj2{80}(3,:)-par.wTable{80}(3,:));
(err2-err1)/err2

err2 = par.wTable{80}(3,end) - par.wrTraj2{80}(3,end);
err2 = par.wnomplotTable{80}(3,end) - par.wrTraj2{80}(3,end);
(err2-err1)/err2
(err1-err2)/err1