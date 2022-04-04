%% Load system matrices to workspace

load fp_lin_matrices_fit3.mat

%% Characterizing model

eig_A = eig(A);

%% Considering controllability

Co      = ctrb(A,B);
Co_rank = rank(Co);

%% Considering observability

Ob      = obsv(A,C);
Ob_rank = rank(Ob);

%% Plotting bode diagram

[n,d] = ss2tf(A,B,C,D);

H1    = tf(n(1,:),d);
H2    = tf(n(2,:),d);

bode(H1, H2)

%% Simulate controller

clear;
load fp_lin_matrices_fit3.mat

Qr = diag([10,0,1,0,0]);
Rr = 1;

K = lqr(A, B, Qr, Rr); % feedback gain

x0 = [0.1 0 0 0 0]'; % Initial condition

D = [0 0 0 0 0]';
C = diag([1,1,1,1,1]);

%T=2; % Time duration of the simulation
%sim('statefdbk',T);

%% Poles of the controller

sys_pols = eig(A-B*K);

%% Simulate controller with observer

clear
load fp_lin_matrices_fit3.mat

Qr = diag([1,0,1,0,0]);
Rr = 0.01;

K = lqr(A, B, Qr, Rr); % Feedback gain

x0 = [0.1 0 0 0 0]'; % Initial condition

G = eye(size(A)); %Gain of the process noise
Qe = eye(size(A))*10; %Variance of process errors
Re = eye(2); %Variance of measurement errors
L = lqe(A, G, C, Qe, Re); %Calculate estimator gains

%T=2; % Time duration of the simulation
%sim('statefdbk_w_observer',T);

%% Automated simulation

Rr = 0.01;

s = 9;
T=5;

% Simulate and save data
for i = 3:s
    for j = 3:s

        Qr = diag([10^i,0,10^j,0,0]);
        K = lqr(A, B, Qr, Rr);
        
        sim('statefdbk_w_observer',T);
        load output.mat
        filename = ['q1_' num2str(i,'%d') '_q3_' num2str(j,'%d')];

        save(filename,'y');

    end
end

%% Automated evaluation of performance

RMSE_alpha = zeros(s,s);
RMSE_beta  = zeros(s,s);

for i = 3:s
    for j = 3:s
        filename = ['q1_' num2str(i,'%d') '_q3_' num2str(j,'%d') '.mat'];
        load(filename)
        delete(filename)

        time  = y(1,:);
        alpha = y(2,:);
        beta  = y(3,:);

        n = length(time);

        for k = 1:n
            RMSE_alpha(i, j) = RMSE_alpha(i, j) + abs(alpha(k));
            RMSE_beta(i, j)  = RMSE_beta(i, j) + abs(beta(k));
        end
         RMSE_alpha(i,j) = (RMSE_alpha(i,j));
         RMSE_beta(i,j)  = (RMSE_beta(i,j));
    end
end
