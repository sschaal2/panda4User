function myFFTplot(Fs,x, varargin)

if(length(varargin) == 0)
    color = 'b'
else
    color = varargin{1}
end

% Use next highest power of 2 greater than or equal to 
% length(x) to calculate FFT. 
NFFT= 2^(nextpow2(length(x))); 


% Take fft, padding with zeros so that length(FFTX) is equal to 
% NFFT 
FFTX = fft(x,NFFT); 


% Calculate the numberof unique points 
NumUniquePts = ceil((NFFT+1)/2); 


% FFT is symmetric, throw away second half 
FFTX = FFTX(1:NumUniquePts); 
MX = abs(FFTX); 


% Take the magnitude of fft of x 
MX = abs(FFTX); 


% Scale the fft so that it is not a function of the 
% length of x 
MX = MX/length(x); 


% Take the square of the magnitude of fft of x. 
MX = MX.^2; 


% Multiply by 2 because you 
% threw out second half of FFTX above 
MX = MX*2; 


% DC Component should be unique. 
MX(1) = MX(1)/2; 


% Nyquist component should also be unique.
if ~rem(NFFT,2) 
   % Here NFFT is even; therefore,Nyquist point is included. 
   MX(end) = MX(end)/2; 
end 


%This is an evenly spaced frequency vector with 
% NumUniquePts points. 
f = (0:NumUniquePts-1)*Fs/NFFT; 


% Generate the plot, title and labels. 
plot(f,MX, 'color', color); 
xlabel('Frequency (Hz)'); 
ylabel('Power'); 
%axis([0 2 0 max(MX)]);