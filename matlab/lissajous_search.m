R=[];
for ratio=1:0.01:2
    for beta=0.5:0.01:1
       X=lissajou(ratio,beta);
       h=histogram(X,20);
       e=-sum(sum(h.*log(h)));
       e=entropy(h);
       R=[R;[ratio,beta,e]];
       endls
end
figure;
plot3(R(:,1),R(:,2),R(:,3),'*');