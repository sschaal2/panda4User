R=[];

freq_base=0.3;
amplitude_slow=0.002;
amplitude_fast=0.002;
amplitude_rot=0.02;

count = 0;
res=20;
R=zeros((res+1)^4,6);

prim = [0.011,0.03,0.0017,0.0037]*0;


for freq_ratio=1.:1/res:2
    for convex_beta=0.5:0.5/res:1
        for freq_ratio_rot=2.:2/res:4
            for convex_freq_ratio=0.1:0.4/res:0.5
                
                % run lissajous pattern
                X=lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot,...
                   freq_ratio+prim(1), freq_ratio_rot+prim(2), convex_beta+prim(3), ...
                       convex_freq_ratio+prim(4),3,20,0);
                
                % create histogram
                H = floor((X + ones(length(X),1)*[amplitude_slow amplitude_fast amplitude_rot])./ ...
                    ((ones(length(X),1)*2*[amplitude_slow amplitude_fast amplitude_rot])/res));
                inds = find(H<=0);
                H(inds)=1;
                inds = find(H>res);
                H(inds)=res;
                
                h=zeros(res,res,res);
                for i=1:length(H)
                 h(H(i,1),H(i,2),H(i,3)) = 1 + h(H(i,1),H(i,2),H(i,3));
                end
                h = h/length(H);
                h1 = sum(h,3); % only x-y entropy
                
                moment_arm = 0.015;
                e1=-sum(sum(h1.*log(h1+1.e-6)));
                e=-sum(sum(sum(h.*log(h+1.e-6))));
                e = 0*0.75*e+e1;
                XX = [X(:,1)+X(:,3)*moment_arm X(:,2)];
                Xd=diffnc(XX(:,1:2),0.01);
                max_vel = sqrt(max(sum(Xd.^2,2)));
                count = count+1;
                R(count,:)=[freq_ratio+prim(1),freq_ratio_rot+prim(2),convex_beta+prim(3),convex_freq_ratio+prim(4),e,max_vel];
                if rem(count,10000) == 0
                    disp(count);
                end
            end
        end
    end
end
thres = 0.01;
T=R(:,5);
exclude = find(R(:,6)>thres);
T(exclude)=0;
[RR,i]=sort(T,'descend');
RR=R(i,:);
RR(1:10,:)
freq_ratio=RR(1,1);
freq_ratio_rot=RR(1,2);
convex_beta=RR(1,3);
convex_freq_ratio=RR(1,4);
lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio, freq_ratio_rot, convex_beta, convex_freq_ratio,3,10,.1);
lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio, freq_ratio_rot, convex_beta, convex_freq_ratio,3,20,.1);
