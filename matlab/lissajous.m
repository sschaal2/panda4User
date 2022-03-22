function [X]=lissajous( freq_base, ...
    amplitude_slow, ...
    amplitude_fast, ...
    amplitude_rot, ...
    freq_ratio, ...
    freq_ratio_rot, ...
    convex_beta, ...
    convex_freq_ratio,...
    trans_duration, ...
    duration,...
draw)
%
% freq_base: base frequency (i.e., freq of slow sine wave)
% amplitude_slow: amplitude of slow sine of Lissajous
% amplitude_fast: amplitude of fast sine of Lissajous
% amplitude_rot: rotational amplitude around normal
% freq_ratio: fast to slow ratio, should be > 1
% freq_ratio_rot: rotation frequence relative to slow sine, usualy > 1
% convex_beta in [0,1]: convex combination coeff. of superimposed extra slow sine wave
% convex_freq_ratio: freq. ratio of extra slow sine wave, should be < 1

% timing
dt       = 0.01; % in seconds
delta = 0; % phase offset

% the transient is a linear ramp to increase the amplitdes from zero to
% full
trans=0;

omega = 2*pi*freq_base;

t=(0:duration/dt)'*dt;
trans=t/trans_duration;
trans=min(1,trans);
x = amplitude_slow * (convex_beta * sin(omega * t + delta) + ...
    (1 - convex_beta) * sin(omega * convex_freq_ratio * t)).*trans;
y = amplitude_fast * sin(omega * freq_ratio * t).*trans;
z = amplitude_rot * sin(omega * freq_ratio_rot * t).*trans;
X = [x,y,z];

if draw
    figure;
    plot(X(:,1),X(:,2));
    hold on;
    quiver(X(:,1),X(:,2),cos(X(:,3)),sin(X(:,3)),0.5);
    hold off;
end


