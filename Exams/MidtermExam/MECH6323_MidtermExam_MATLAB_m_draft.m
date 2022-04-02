%% MECH 6323 - Robust Control - Midterm Exam
% Author: Jonas Wagner
% 
% Date: 2022-04-02

clear
close all
%% Problem 4 - Plane Autopilot System
% -------------------------------------------------------------------------------------------------------
%% Controller Definition
% Parameters

K_a = -1.5e-3;
K_q = -0.32;
a_z = 2;
a_q = 6;
% State Matrices

A_C = [ 0           0
        K_q*a_q   0];
B_C = [ K_a*a_z           0
        K_a*K_q*a_q     K_q*a_q];
C_C = [ K_q    1];
D_C = [ K_a*K_q       K_q];
% System Definition

sys_C = ss(A_C, B_C, C_C, D_C)
tf_C = tf(sys_C)
% -------------------------------------------------------------------------------------------------------
%% Plant Definition
% Parameters

V = 886.78;
zeta = 0.6;
omega = 113;
% Uncertain Coeficients
% Nominal Values

Z_alpha_0 = -1.3046;
Z_delta_0 = -0.2142;
M_alpha_0 = 47.7109;
M_delta_0 = -104.83436;
% Uncertain Dynamics

Z_alpha = @(k, delta) Z_alpha_0 * (1 + k * delta);
Z_delta = @(k, delta) Z_delta_0 * (1 + k * delta);
M_alpha = @(k, delta) M_alpha_0 * (1 + k * delta);
M_delta = @(k, delta) M_delta_0 * (1 + k * delta);
% System Matrices

A_P = @(Z_alpha, Z_delta, M_alpha, M_delta) [ 
    Z_alpha	1	Z_delta	    0
    M_alpha	0	M_delta	    0
    0	    0	0	        1
    0	    0	-omega^2	-2*zeta*omega
];
B_P = [
    0
    0
    0
    omega^2
];
C_P = @(Z_alpha, Z_delta) [
    V*Z_alpha   0   V*Z_delta   0
    0           1   0           0
];
D_P = [
    0
    0
];
% Nominal System
% Nominal System Matrices

A_P_0 = A_P(Z_alpha_0, Z_delta_0, M_alpha_0, M_delta_0)
% double(vpa(subs(A_P, ...
%     [Z_alpha, Z_delta, M_alpha, M_delta], ...
%     [Z_alpha_0, Z_delta_0, M_alpha_0, M_delta_0]), 4));
B_P_0 = B_P
% double(vpa(subs(B_P, ...
%     [Z_alpha, Z_delta, M_alpha, M_delta], ...
%     [Z_alpha_0, Z_delta_0, M_alpha_0, M_delta_0]), 4));
C_P_0 = C_P(Z_alpha_0, Z_delta_0) 
% double(vpa(subs(C_P, ...
%     [Z_alpha, Z_delta, M_alpha, M_delta], ...
%     [Z_alpha_0, Z_delta_0, M_alpha_0, M_delta_0]), 4));
D_P_0 = D_P
% double(vpa(subs(D_P, ...
%     [Z_alpha, Z_delta, M_alpha, M_delta], ...
%     [Z_alpha_0, Z_delta_0, M_alpha_0, M_delta_0]), 4));
% Nominal State-space System

sys_P_0 = ss(A_P_0, B_P_0, C_P_0, D_P_0)
% Nominal Transfer Function

tf_P_0 = tf(sys_P_0)
% Uncertain System Dynamics
% Uncertain Matrices

A_P = @(k, Delta) A_P(  Z_alpha(k, Delta(1)), ...
                        Z_delta(k, Delta(2)), ...
                        M_alpha(k, Delta(3)), ...
                        M_delta(k, Delta(4)));
C_P = @(k, Delta) C_P(  Z_alpha(k, Delta(1)), ...
                        Z_delta(k, Delta(2)));
% Uncertain System

sys_P = @(k, Delta) ss(A_P(k, Delta), B_P, C_P(k, Delta), D_P);
% -------------------------------------------------------------------------------------------------------
%% Feedback System Definition
% Open Loop System: $L(s) = C(s) P(s)$
% Nominal

sys_L_0 = series(sys_C, sys_P_0)
% Uncertain

sys_L = @(k, Delta) series(sys_C, sys_P(k, Delta));
% Closed Loop System: $S(s) = \frac{C(s) P(s)}{1 + C(s) P(s)}$
% Nominal

sys_S_0 = feedback(sys_L_0, eye(size(C_P_0,1)))
% Uncertain

sys_S = @(k, Delta) feedback(sys_L(k,Delta), eye(size(C_P_0,1)));
%% -------------------------------------------------------------------------------------------------------
%% (a)

eig_S_0 = eig(sys_S_0)
%% 
% Since $\mathcal{R}(\lambda_i) < 0 \ \forall_{i}$, the closed loop system $S(s)$ 
% is stable.

sys_S_stable = isstable(sys_S_0)
%% (b) 
% Random Testing Code

i_worst = 1;
k_try = 10;
N_samples = 10000;
Delta_data = 2 * rand([4, N_samples]) - 1;
for i = 1:N_samples
    % Better Implimentation
    sys_S_test = @(k) sys_S(k, Delta_data(:,i));
    if isstable(sys_S_test(k_try))
        break
    else
        while ~isstable(sys_S_test(k_try))
            k_try = 0.99 * k_try;
            i_worst = i;
        end
    end
    % Alternative (as described in the problem itself)
%     Delta = Delta_data(:,i);
%     sys_S_unstable = true;
%     while sys_S_unstable
%         sys_S_test = sys_S(k_try, Delta);
%         if max(real(eig(sys_S_test))) >= 0
%             k_try = 0.99 * k_try;
%             i_worst = i;
%         else
%             sys_S_unstable = false;
%         end
%     end
end
% Random Testing Results

k_bar = k_try
Delta_worst = Delta_data(:,i)
%% 
% Since this gets randomized everytime, $\overline{k}$ changes but often gets 
% down below 1 to 0.7-ish, but also stays at 10 sometimes.
% Why must $k_{max} \leq \overline{k}$?
% Well, many reasons. We know that $k_{max}$is the largest possible $k$ that 
% maintains stability of the closed-loop system, and therefore all unstabilizing 
% $k$ due to some disturbance would be greater then the lowest-upper bound on 
% $k$, i.e. $\overline{k} \geq \sup_{k} S(s)$ stable. Therefore, $k_{max} =  \sup_{k} 
% S(s) \leq \overline{k}$.
%% -------------------------------------------------------------------------------------------------------
%% (c) 
% Uncertain System
% Bounded Uncertainty

delta_1_u = ureal('delta_1', 0);
delta_2_u = ureal('delta_2', 0);
delta_3_u = ureal('delta_3', 0);
delta_4_u = ureal('delta_4', 0);

Delta_u = [
    delta_1_u
    delta_2_u
    delta_3_u
    delta_4_u
];
% Uncertain SS system

sys_S_u = @(k) sys_S(k, Delta_u);
% Uncertain System Frequency Respons

w_min = -2;
w_max = 4;
freqs = logspace(w_min,w_max,100);
sys_S_u_k1 = sys_S_u(1);
usysfrd = ufrd(sys_S_u_k1, freqs);
bode(usysfrd,'r', usysfrd.NominalValue, 'b+')
% robostab

opts = robOptions(  'Display','on', ...
                    'VaryFrequency','on',...
                    'Sensitivity','on');
[stabmarg, destabunc, report] = robstab(sys_S_u_k1, opts)
%% 
% From this we have $k_{max} \approx 0.6045$ to result in $k * \Delta = 0.6045 
% * [-1 \ -1 \ 1 \ -1]^T$.
% 
% This value is consistant with the numerical results as it is below, but not 
% too small by comparrision to what is expected.
% 
% Additionally, we know that the critical frequency that this is occurring at 
% is 
%% -------------------------------------------------------------------------------------------------------
%% (d)

k_max = 0.6045;
sys_S_critical = sys_S(k_max, [-1; -1; 1; -1])
tf_S_critical = tf(sys_S_critical)
eig_S_critical = eig(sys_S_critical)
%% 
% As can be seen by the eigenvalues at $\lambda_{4,5} = 0.00 \pm j 3.44$, the 
% poles of the system cross the$j \omega$-axis to become unstable at $k_{max} 
% \approx 0.6045$ with $\omega \approx 3.44$. This confirms the robostab estimates 
% to force the system to become unstable at the maximum perturbation of $\Delta 
% = [-1 \ -1 \ 1 \ -1]^T$.