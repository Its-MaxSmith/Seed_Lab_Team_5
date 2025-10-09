%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
load('rightStepData.mat')
%% Define motor parameters
K=4.12; % DC gain [rad/Vs]
sigma=14; % time constant reciprocal [1/s]

% RIGHT K = 4.12
%       sigma = 14

%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motorControlRight')
%
% run the simulation
%
out=sim('motorControlRight');
%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'linewidth',2)
hold on
plot(out.DesiredVelocity,'--','linewidth',2)
legend('Velocity','DesiredVelocity','location','southeast')