clc;
close all;
clear all;

M = readmatrix('qcm.txt');

a = 1;
b = 1;
n = 11;
matrix_length = size(M, 1);
num_groups = matrix_length / n;

K = [];
L = [];
N = [];
Derivative = [];

for group = 1:num_groups
    start_index = (group - 1) * n + 1;
    end_index = group * n;
    
    S = M(start_index+1:end_index, 1);
    Z = M(start_index+1:end_index, 2);
    V = M(start_index+1:end_index, 3);  
     
    F = median(S);
    P = median(Z);
    H = mean(V);
    
    K = [K, F];
    L = [L, P];
    N = [N, H];
end 
 
Mg = K * (3.3 / 4095);
Ph = L * (3.3 / 4095);
Mag = ((Mg * 1000) - 930) / 30;
Phase = ((Ph * 1000) - 1800) / 10;

plot(N, Mag);
title("Magnitude")
xlabel('Frequency')
ylabel('dB')
figure
plot(N, Phase);
title("Phase")
xlabel('Frequency')
ylabel('Degree') 


for i = 3:numel(Mag)-2
    x_forward = mean(Mag(i+1:i+2));
    x_backward = mean(Mag(i-2:i-1));
    y_forward = mean(N(i+1:i+2));
    y_backward = mean(N(i-2:i-1));
    
    Derivative(i) = (x_forward - x_backward) / (y_forward - y_backward);
end


for i = 1:2
    Derivative(i) = (mean(Mag(i+1:i+2)) - mean(Mag(1:i))) / (mean(N(i+1:i+2)) - mean(N(1:i)));
end

for i = numel(Mag)-1:numel(Mag)
    Derivative(i) = (mean(Mag(end-1:end)) - mean(Mag(end-4:end-3))) / (mean(N(end-1:end)) - mean(N(end-4:end-3)));
end

figure
plot(N, Derivative);
title("Derivative using CFD")
xlabel('Frequency')
ylabel('Slope')
