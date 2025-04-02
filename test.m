%% Integrated Solved Example: Power Analysis for Blood Glucose Sensor Comparison
% ---------------------------------------------------------------
% Problem Statement:
% You are designing a study to compare two blood glucose sensors:
%
%   - Sensor A (Standard): Known mean reading μ₀ = 120 mg/dL and standard deviation σ = 10 mg/dL.
%   - Sensor B (Prototype): Hypothesized to have a mean reading about 5 mg/dL lower 
%     (i.e., μ₁ ≈ 115 mg/dL).
%
% You plan to use a two-sided two-sample t-test (assuming equal variances) to test:
%   H0: μ_A = μ_B   versus   H_a: μ_A ≠ μ_B.
%
% Study design criteria for the Equal Variances Scenario:
%   - Significance level: α = 0.05.
%   - Desired power: 90% (i.e., 1 - β = 0.90).
%
% Objectives (Equal Variances):
% 1. Determine analytically (using MATLAB's sampsizepwr) the minimum sample size per group
%    needed to achieve 90% power to detect a 5 mg/dL difference.
% 2. Perform a Monte Carlo simulation to estimate empirical power as the sample size per group varies
%    from 5 to 120 (in steps of 5).
% 3. Plot both the analytical power (from sampsizepwr) and the empirical power versus sample size,
%    and visually mark (using xline and yline) the point where power first reaches 90%.
%
% Extension – Unequal Variances Scenario:
% Now assume that Sensor A has σₐ = 10 mg/dL, but Sensor B has a higher variability with σ_b = 15 mg/dL.
% In this case, we use Welch’s t-test (which does not assume equal variances) to estimate empirical power.
% MATLAB’s sampsizepwr does not directly support unequal variances for a two-sample t-test,
% so we rely on Monte Carlo simulation. The sample size range is extended to 250 per group.
% ---------------------------------------------------------------

clear;close all; clc;

%% Section 1: Equal Variances Scenario

% Parameters for equal variances:
mu0    = 120;      % Mean for Sensor A (mg/dL)
mu1    = 115;      % Mean for Sensor B (mg/dL)
sigma  = 10;       % Common standard deviation (mg/dL)
alpha  = 0.05;     % Significance level
Nrep   = 1000;     % Number of Monte Carlo iterations per sample size

% Define sample sizes per group: from 5 to 120 in steps of 5.
nRange = 5:5:120;
empPower = zeros(size(nRange));  % Preallocate for empirical power

% Monte Carlo Simulation (Equal Variances)
for i = 1:length(nRange)
    n = nRange(i);
    rejectCount = 0;
    for r = 1:Nrep
        dataA = normrnd(mu0, sigma, [n, 1]);  % Sensor A: N(120, 10^2)
        dataB = normrnd(mu1, sigma, [n, 1]);  % Sensor B: N(115, 10^2)
        [~, p_val] = ttest2(dataA, dataB, 'Alpha', alpha, 'Vartype', 'equal');
        if p_val < alpha
            rejectCount = rejectCount + 1;
        end
    end
    empPower(i) = rejectCount / Nrep;
end

% Analytical Power Calculation using sampsizepwr
analyticalPower = sampsizepwr('t2', [mu0 sigma], mu1, [], nRange, 'Alpha', alpha);
requiredSampleSize = sampsizepwr('t2', [mu0 sigma], mu1, 0.90, [], 'Alpha', alpha);

% Plot Empirical vs. Analytical Power (Equal Variances)
figure;
plot(nRange, empPower, 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor','b'); 
hold on;
plot(nRange, analyticalPower, 'r--', 'LineWidth', 1.5);
xlabel('Sample Size per Group');
ylabel('Power');
title('Power Analysis (Equal Variances)');
legend({'Empirical (Simulation)', 'Analytical (sampsizepwr)'}, 'Location', 'southeast');
grid on;
% Add horizontal and vertical markers in black, and exclude from legend.
yline(0.90, 'k--', 'LineWidth', 1.5, 'HandleVisibility','off');
idx90 = find(empPower >= 0.90, 1, 'first');
if ~isempty(idx90)
    xline(nRange(idx90), 'k--', 'LineWidth', 1.5, 'HandleVisibility','off');
    disp('Equal Variances Scenario:');
    disp("Empirical simulation: 90% power is first reached at a sample size per group of " + nRange(idx90) + ".");
else
    disp("Equal Variances Scenario: 90% power was not reached within the tested sample size range.");
end
disp("Analytical sample size required for 90% power: " + ceil(requiredSampleSize) + ".");
