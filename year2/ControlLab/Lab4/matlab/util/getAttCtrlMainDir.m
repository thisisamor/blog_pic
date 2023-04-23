function [ str_log_dir_full ] = getAttCtrlMainDir()
%GETMAINDIR Summary of this function goes here
%   Detailed explanation goes here

% recover the directory in which this file is located (i.e., the root
% directory of the repo)
str_file_path = mfilename('fullpath');
[str_file_dir, ~, ~] = fileparts(str_file_path);
str_root_dir = str_file_dir;

str_log_dir_full = fullfile(str_root_dir, '..');


end

