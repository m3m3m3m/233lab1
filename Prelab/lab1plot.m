function [] = lab1plot(Vo, R, C, f, step)
% TIMEPLOT: Plot voltage waveforms in the time domain from -1ms to 1ms
% USAGE: timeplot(Vo, R, C, charge);
%        DC: Nonzero DC voltage value
%        R: Resistor value (ohms)
%        C: Capacitor value (microFarads)
%        f: Signal frequency (Hertz)
%        step:
%           Prelab #3:  place "1" here
%           Prelab #5:  place "2" here
%           Prelab #7:  place "3" here
%           Prelab #10: place "4" here
%           Prelab #11: place "5" here
%           Prelab #12: place "6" here

% check for incomplete inputs
if nargin < 5
    error('Invalid number of inputs. Try again.')
end

% create time vector
t = -1:0.001:1;

% charge and discharge equations
v1 = Vo.*(1 - exp(-(t.*1e-3)./(R*C)));
v1(1:floor(length(t)/2)) = 0;
v2 = Vo.*exp(-(t.*1e-3)./(R*C));
v2(1:floor(length(t)/2)) = Vo;

% Prelab #3
if step == 1
    % output
    v = v1;
    % input
    vin1 = zeros(1,floor(length(t)/2));
    vin2 = Vo.*ones(1,floor(length(t)/2)+1);
    vin = [vin1 vin2];
    
% Prelab #5
elseif step == 2
    % output
    v = v2;
    % input
    vin1 = zeros(1,floor(length(t)/2));
    vin2 = Vo.*ones(1,floor(length(t)/2)+1);
    vin = [vin2 vin1];

% Prelab #7
elseif step == 3
    % input
    t1 = 0:0.001:8;
    vin2 = Vo.*zeros(1,floor(length(t1)./4));
    vin3 = Vo.*ones(1,floor(length(t1)./4));
    vin4 = Vo.*ones(1,floor(length(t1)./4)+1);
    vin = [vin3 vin2 vin4 vin2];
    % output
    vout1 = v1(floor(length(t)/2)+1:end);
    vout2 = zeros(1,floor(length(t)/2));
    v = [vout1 v2(1:end-1) v1(1:end-1) v2(1:end-1) vout2];
    t = t1;

% Prelab #10
elseif step == 4
    t = 0:0.001:5;
    vin = Vo.*cos(2*pi*f.*(t.*1e-3));
    mag = Vo/(1+R^2*C^2*4*pi^2*f^2);
    v = mag.*(cos(2*pi*f.*(t.*1e-3)) + R*C*2*pi*f.*sin(2*pi*f.*(t.*1e-3)));

% Prelab #11
elseif step == 5
    freq = 10:1e6;
    mag = Vo./sqrt(1+R^2*C^2*4*pi^2.*freq.^2);

% Prelab #12
elseif step == 6
    freq = 10:1e6;
    mag = (Vo*2*pi*R*C.*freq)./sqrt(1+R^2*C^2*4*pi^2.*(freq.^2));
end

% Plot v_in and v_out
if step < 5
    figure()
    plot(t,vin,'k','LineWidth',2)
    hold on
    plot(t,v,'r','LineWidth',2)
    legend('v_{in}(t)','v_{out}(t)')
    if step < 4
        ylim([0,1.2*Vo])
    else
        ylim([-Vo,Vo])
    end
    title('RC Circuit Time Response')
    xlabel('time (ms)')
    ylabel('voltage (V)')
else 
    figure()
    semilogx(freq,mag,'r','LineWidth',2)
    title('RC Circuit Frequency Response')
    xlabel('frequency (Hz)')
    ylabel('voltage (V)')
end

end

