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

% Compute derivative
for i = 1:numel(Mag)-1
    if i < numel(Mag) - 5
        % For points other than the last five points we need to compute the average of the next 5 points
        x = mean(Mag(i+1:i+5));
    else
        x = mean(Mag(i+1:end));
    end
    
    if i > 5
        y = mean(Mag(i-5:i-1));
    else
        y = mean(Mag(1:i-1));
    end    
    Derivative(i) = (x - y) / (N(i+1) - N(i));
end

% For the last point we can't find derivative
Derivative(end+1) = Derivative(end);

figure
plot(N, Derivative);
title("Derivative")
xlabel('Frequency')
ylabel('Slope')

