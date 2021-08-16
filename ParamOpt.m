clear all;
close all;
clc;

%% Parameter Optimization

% Constants:
% I want to sample a 100 us period (1e-4 seconds) starting 10 us before a
% trigger event and ending 90 us after. The sample rate I have set us to is
% 3.2e-8 samples per second, or 31.25 MS/s. This can be adjusted but it is
% currently hardcoded in scopeTest.m The bounds of the sample periods and
% buffer size will be defined thorugh this.

sample_period = 3.2e-8;
samples_per_second = 1 / sample_period;
samples_in_desired_window = samples_per_second * 1e-4;
trigg_freq = 100; % Hz

min_frac = 0.001;
max_frac = 5;
min_size = 3125;


