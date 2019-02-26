load('debuggingFMM.mat');

g.min = [-5.0;-3.5];
g.max = [5.0;3.5];
g.xs{1} = single(g.xs{1});
g.xs{2} = single(g.xs{2});
compute_value(g,target, speed);