% frequency (in Hz)
f = 440;
% sampling frequency (in Hz)
fs = 10000;
% amplitude
a = 1;

% generate sample
t = 0 : 512;
x = a / 2 * sin( t * f / fs * 2 * pi );

% plot
figure(1);
stem(t,x,'Color','r'); hold on;
line(t,x,'Color','b');