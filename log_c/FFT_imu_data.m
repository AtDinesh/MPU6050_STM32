clear all;
close all;

Fs = 1000;
T = 1/Fs;
static = load('static300.dat');
L = size(static,1);
t = (0:L-1)*T;

Ax = static(:,2);
Ay = static(:,3);
Az = static(:,4);
Gx = static(:,5);
Gy = static(:,6);
Gz = static(:,7);

%% Acelerometer
figure('Name','FFT acceleration','NumberTitle','off');
subplot(3,1,1);
plot(1000*t(1:L),Ax(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Ax(t)');

hold on;

subplot(3,1,2);
plot(1000*t(1:L),Ay(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Ay(t)');

subplot(3,1,3);
plot(1000*t(1:L),Az(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Az(t)');

%% Gyroscope
figure('Name','FFT gyroscope','NumberTitle','off');
subplot(3,1,1);
plot(1000*t(1:L),Gx(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Gx(t)');

hold on;

subplot(3,1,2);
plot(1000*t(1:L),Gy(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Gy(t)');

subplot(3,1,3);
plot(1000*t(1:L),Gz(1:L));
title('Signal Corrupted with Zero-Mean Random Noise');
xlabel('t (milliseconds)');
ylabel('Gz(t)');

%% Histogram

figure('Name','histogram acceleration','NumberTitle','off');
subplot(3,1,1);
title('Histogram Ax');
hax = histogram(Ax);

subplot(3,1,2);
title('Histogram Ay');
hay = histogram(Ay);

subplot(3,1,3);
title('Histogram Az');
haz = histogram(Az);

figure('Name','histogram gyroscope','NumberTitle','off');
subplot(3,1,1);
title('Histogram Gx');
hgx = histogram(Gx);

subplot(3,1,2);
title('Histogram Gy');
hgy = histogram(Gy);

subplot(3,1,3);
title('Histogram Gz');
hgz = histogram(Gz);

%% distribution

pd_ax = fitdist(Ax,'Normal');
pd_ay = fitdist(Ay,'Normal');
pd_az = fitdist(Az,'Normal');

pd_gx = fitdist(Gx,'Normal');
pd_gy = fitdist(Gy,'Normal');
pd_gz = fitdist(Gz,'Normal');

%% plot pdf distributions

% y_ax = pdf(pd_ax, Ax);
% y_ay = pdf(pd_ay, Ay);
% y_az = pdf(pd_az, Az);
% 
% y_gx = pdf(pd_gx, Gx);
% y_gy = pdf(pd_gy, Gy);
% y_gz = pdf(pd_gz, Gz);
% 
% figure('Name','PDF of distributions','NumberTitle','off');
% subplot(3,1,1);
% title('PDF Ax');
% plot(Ax,y_ax,'LineWidth',2);
% 
% subplot(3,1,2);
% title('PDF Ay');
% plot(Ay,y_ay,'LineWidth',2);
% 
% subplot(3,1,3);
% title('PDF Az');
% plot(Az,y_az,'LineWidth',2);
% 
% figure('Name','PDF of distributions','NumberTitle','off');
% subplot(3,1,1);
% title('PDF Gx');
% plot(Gx,y_gx,'LineWidth',2);
% 
% subplot(3,1,2);
% title('PDF Gy');
% plot(Gy,y_gy,'LineWidth',2);
% 
% subplot(3,1,3);
% title('PDF Gz');
% plot(Gz,y_gz,'LineWidth',2);