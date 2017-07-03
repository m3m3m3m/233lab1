% EE 233 Lab 1: RC Circuits
% Template for waveform plotting
% Author: Matt Jones

clear all; close all; clc

% Circuit parameters
R = 10000;      % Resistance in ohms
C = 0.01e-6;    % Capacitance in Farads
Vo = 5;         % Positive voltage constant
f = 1000;       % Signal frequency in Hertz

% Use LAB1PLOT function to plot waveforms
% Prelab #3:    Last digit is "1"
% Prelab #5:    Last digit is "2"
% Prelab #7:    Last digit is "3"
% Prelab #10:   Last digit is "4"
% Prelab #11:   Last digit is "5"
% Prelab #12:   Last digit is "6"
lab1plot(Vo,R,C,f,6);
