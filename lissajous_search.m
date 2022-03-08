R=[];

freq_base=0.3;
amplitude_slow=5;
amplitude_fast=5;
amplitude_rot=2;

count = 0;
res=20;
R=zeros((res+1)^4,6);


for freq_ratio=1.:1/res:2
    for convex_beta=0.5:0.5/res:1
        for freq_ratio_rot=2.:2/res:4
            for convex_freq_ratio=0.1:0.4/res:0.5
                
                % run lissajous pattern
                X=lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio+0.011, freq_ratio_rot+0.03, convex_beta+0.0017, convex_freq_ratio+0.0037,0);
                
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
                
                e=-sum(sum(sum(h.*log(h+1.e-6))));
                Xd=diffnc(X(:,1:2),0.01);
                max_vel = sqrt(max(sum(Xd.^2,2)));
                count = count+1;
                R(count,:)=[freq_ratio+0.011,freq_ratio_rot+0.03,convex_beta+0.0017,convex_freq_ratio+0.0037,e,max_vel];
                if rem(count,10000) == 0
                    disp(count);
                end
            end
        end
    end
end
thres = 13;
T=R(:,5) + 0.0*(1-R(:,6)/max(R(:,6)));
exclude = find(R(:,6)>thres);
T(exclude)=0;
[RR,i]=sort(T,'descend');
RR=R(i,:);
RR(1:10,:)
freq_ratio=RR(1,1);
freq_ratio_rot=RR(1,2);
convex_beta=RR(1,3);
convex_freq_ratio=RR(1,4);
lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio, freq_ratio_rot, convex_beta, convex_freq_ratio,.1);
