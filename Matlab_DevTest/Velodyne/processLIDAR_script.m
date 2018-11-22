% PROCESSLIDAR_SCRIPT   Generate MEX-function processLIDAR_mex from
%  processLIDAR.
% 
% Script generated from project 'processLIDAR.prj' on 26-Jan-2018.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.MexCodeConfig'.
cfg = coder.config('mex');
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.EnableJIT = true;

%% Define argument types for entry-point 'processLIDAR'.
ARGS = cell(1,1);
ARGS{1} = cell(2,1);
ARGS{1}{1} = coder.typeof(single(0),[16000   3],[1 0]);
ARGS{1}{2} = coder.typeof(0);

%% Invoke MATLAB Coder.
codegen -config cfg processLIDAR -args ARGS{1}

