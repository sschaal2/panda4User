l = dir('stat*.txt');
for i=1:length(l),
    vl_stats(l(i).name);
end