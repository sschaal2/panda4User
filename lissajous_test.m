freq_base     =0.3;
amplitude_slow=0.002;
amplitude_fast=0.002;
amplitude_rot =0.02;
freq_ratio = 4/3;
freq_ratio_rot = 4;
convex_beta = 1;
convex_freq_ratio = 0.43;


X = lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio, freq_ratio_rot, convex_beta, convex_freq_ratio,3,20,1);
X = lissajous(freq_base, amplitude_slow, amplitude_fast, amplitude_rot, freq_ratio, freq_ratio_rot, convex_beta, convex_freq_ratio,3,10,1);

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

moment_arm = 0.015;
e=-sum(sum(sum(h.*log(h+1.e-6))))

h1=sum(h,3);
e1=-sum(sum(h1.*log(h1+1.e-6)))

XX = [X(:,1)+X(:,3)*moment_arm X(:,2)];
Xd=diffnc(XX(:,1:2),0.01);
max_vel = sqrt(max(sum(Xd.^2,2)))

if 0
    figure;
    for i=1:res,
        bar3(h(:,:,i));
        pause,
    end
end