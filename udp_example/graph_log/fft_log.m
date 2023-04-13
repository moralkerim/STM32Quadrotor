function  [fft_mag,f] = fft_log(enum,fs,log_mat)
    N = length(log_mat(:,enum));
    fft_signal = fft(log_mat(:,enum));
    fft_oneSide = fft_signal(1:N/2);
    f = fs*(0:N/2-1)/N;
    fft_mag = abs(fft_oneSide)/(N/2);
    
    %figure
    plot(f,fft_mag);
    xlabel('Frequency (Hz)');
    ylabel('Amplitude');
    title('Frequency-domain plot');
    hold on;
end

