close all;

y_noise = out.noisy_signal.signals.values;
ti = out.noisy_signal.time;
save('noise_data.mat','y_noise','ti');
data = load('noise_data.mat')
y_noise = data.y_noise;
ti      = data.ti;
Ts = ti(2) - ti(1)
Fs = 1/Ts; 
N = length(y_noise);
Y = fft(y_noise - mean(y_noise));        
f = (0:N-1)*(Fs/N);          
P = abs(Y).^2 / N;           
figure; 
plot(f(1:N/2), P(1:N/2));
xlabel('Frequency (Hz)');
ylabel('Power');
grid on;
title('Noise FFT');