function pid_playground(mode, gainName, values)
%PID_PLAYGROUND  Tune a PID controller on a simulated turret yaw axis.
%
%   pid_playground                                  run once with the gains below
%   pid_playground('sweep','kp',[0.01 0.02 0.04])   overlay several values of one gain
%   pid_playground('imu')                           look at what the simulated IMU gives you
%
% Everything is simulated: the motor, the turret's inertia, and an IMU that
% behaves like the real BNO055 on our robots (100 Hz updates, 1/16 degree
% resolution, noise, and a few milliseconds of delay). Nothing here talks to
% hardware, so you cannot break anything. Change numbers and run it again.
%
% Week 3 - embed training.

% =========================================================================
%  YOUR GAINS - this is the part you edit
% =========================================================================
kp = 5.00;      % proportional   [command per degree of error]
ki = 0.00;      % integral       [command per degree-second]
kd = 0.000;     % derivative     [command per degree-per-second]

DISTURBANCE = 0.00;   % set to 0.20 for Exercise 4 (the chassis starts spinning)
SETPOINT    = 20;     % degrees. Exercise 7 uses 120; put it back to 20 afterwards.
% =========================================================================

if nargin < 1, mode = 'run'; end

cfg = config(SETPOINT, DISTURBANCE);

switch lower(mode)
    case 'run'
        r = simulate(kp, ki, kd, cfg);
        m = metrics(r, cfg);
        report(kp, ki, kd, cfg, m);
        plot_run(r, cfg, m, kp, ki, kd);

    case 'sweep'
        if nargin < 3
            error('pid_playground:sweep', ...
                  'Usage: pid_playground(''sweep'', ''kp'', [0.01 0.02 0.04])');
        end
        sweep(kp, ki, kd, cfg, lower(gainName), values);

    case 'imu'
        show_imu(cfg);

    otherwise
        error('pid_playground:mode', ...
              'mode must be ''run'', ''sweep'' or ''imu''.');
end
end


%% 
%  Config
%  
function c = config(setpoint, disturbance)
% --- the plant: motor + gearbox + turret ---------------------------------
% Command u is normalised to [-1, +1]. Give it a constant command and the yaw
% RATE settles exponentially to K*u; the yaw ANGLE is the integral of that.
c.K   = 250;      % [deg/s] of steady yaw rate at full command
c.tau = 0.10;     % [s], how long the motor takes to reach that rate (63%)
c.uMax = 1.0;     % max motor input

% the IMU
c.fsImu   = 100;      % Hz  - fused yaw update
c.lsb     = 1/16;     % deg - resolution of imu (16 counts / deg)
c.noise   = 0.05;     % deg - 1 sigma of injected measurement noise
c.delay   = 0.008;    % s   - I2C transaction + the sensor's internal filtering

% --- the control loop ----------------------------------------------------
c.fsCtrl  = 1000;     % Hz - code runs every 1 ms
c.nFilter = 40;       % rad/s - low-pass on D term
% --- the test ------------------------------------------------------------
if setpoint == 0
    error('pid_playground:setpoint', ...
          ['SETPOINT must not be 0 - the turret starts at 0 deg, so there ' ...
           'would be no step to respond to. Try 20 (or -20).']);
end
c.setpoint    = setpoint;
c.disturbance = disturbance;
c.tDist       = 1.0;     % s, when the disturbance switches on
c.tEnd        = 2.5;     % s
c.dtSim       = 1e-4;    % s, timestep
c.seed        = 1;       

% --- what counts as a good tune (Exercise 6) -----------------------------
c.maxOvershoot = 15;     % percent
c.maxSettle    = 0.70;   % s
c.maxFinalErr  = 0.40;   % deg
end


%% 
%  Simulation

function r = simulate(kp, ki, kd, c)
n      = round(c.tEnd/c.dtSim);
dtCtrl = 1/c.fsCtrl;
nDelay = round(c.delay/c.dtSim);

t     = (0:n-1)'*c.dtSim;
theta = zeros(n,1);   % TRUE yaw 
omega = zeros(n,1);   % true yaw rate
meas  = zeros(n,1);   % what the IMU reports (what the controller sees)
u     = zeros(n,1);   % motor command (control input)

% IMU state
nSamples   = ceil(c.tEnd*c.fsImu) + 2;
noiseSeq   = c.noise * pseudo_gaussian(nSamples, c.seed);
iSample    = 0;
sensorOut  = 0;                  % newest sample, before the delay
nextSample = 0;                  % when the IMU next publishes
delayLine  = zeros(1, nDelay+1); % shift register

% controller state
integral = 0; dState = 0; prevMeas = 0; firstTick = true;
uNow = 0; nextCtrl = 0;
nTick = 0; uTicks = zeros(ceil(c.tEnd*c.fsCtrl)+2, 1);

th = 0; om = 0;

for k = 1:n
    tk = t(k);

    
    % THE IMU. Generate the sensor reading
    % publishes at 100 Hz, rounds to the nearest 1/16 of a degree, adds noise, has few ms delay
    
    if tk >= nextSample - 1e-12
        nextSample = nextSample + 1/c.fsImu;
        iSample    = iSample + 1;
        sensorOut  = round(th/c.lsb)*c.lsb ...   % quantize to 1/16 of a degree
                     + noiseSeq(iSample);        % add noise
    end
    delayLine = [sensorOut, delayLine(1:end-1)]; % age by one step
    yawMeas   = delayLine(end);                  % measured yaw

    % -------------------------------------------------------------
    % CONTROLLER, running at 1 kHz
    % -------------------------------------------------------------
    if tk >= nextCtrl - 1e-12
        nextCtrl = nextCtrl + dtCtrl;

        % error wrapped into [-180, 180]
        err = mod(c.setpoint - yawMeas + 180, 360) - 180;

        P = kp * err;

        % Derivative of the measurement. 
        % Note: We low-pass filtered (only let low frequencies of the signal through 
        % because differentiating a stair-stepped 100 Hz signal at 1 kHz gives
        % you lots of noise. (Run pid_playground('imu') to see why.)
        if firstTick, prevMeas = yawMeas; firstTick = false; end
        rawD     = -(yawMeas - prevMeas)/dtCtrl;
        prevMeas = yawMeas;
        alpha    = dtCtrl/(dtCtrl + 1/c.nFilter);
        dState   = dState + alpha*(rawD - dState);
        D        = kd * dState;

        % Integrate provisionally so we can decide below whether to keep it.
        iCand = integral + ki*err*dtCtrl;

        uUnsat = P + iCand + D;
        uNow   = max(-c.uMax, min(c.uMax, uUnsat));

        % Anti-windup: only commit the integration if we aren't jammed against
        % the motor limit, or if this error is pulling us back off it.
        if uUnsat == uNow || err*uUnsat < 0
            integral = iCand;
        end

        nTick = nTick + 1;
        uTicks(nTick) = uNow;
    end

    % -------------------------------------------------------------
    % PLANT. omega_dot = (-omega + K*u)/tau,  theta_dot = omega
    % -------------------------------------------------------------
    d  = 0;
    if tk >= c.tDist, d = c.disturbance; end
    uc = max(-c.uMax, min(c.uMax, uNow)) + d;

    om0 = om;
    k1  = (-om0 + c.K*uc)/c.tau;
    k2  = (-(om0 + c.dtSim*k1) + c.K*uc)/c.tau;
    om  = om0 + c.dtSim*0.5*(k1 + k2);
    th  = th  + c.dtSim*0.5*(om0 + om0 + c.dtSim*k1);

    theta(k) = th; omega(k) = om; meas(k) = yawMeas; u(k) = uNow;
end

r = struct('t', t, 'theta', theta, 'omega', omega, 'meas', meas, ...
           'u', u, 'uTicks', uTicks(1:nTick));
end


%% 
%  Deterministic "random" noise
%  A minimal standard generator (Lehmer / minstd) fed through Box-Muller.
%  Not cryptography - just repeatable, identical everywhere, and good enough
%  to look like sensor noise.

function z = pseudo_gaussian(n, seed)
M = 2147483647;                 % 2^31 - 1
s = mod(abs(seed)*7919 + 12345, M);
if s == 0, s = 1; end
u = zeros(1, 2*n);
for i = 1:2*n
    s    = mod(16807*s, M);     % 16807 * (2^31-1) stays well inside 2^53
    u(i) = s/M;
end
u1 = max(u(1:2:end), 1e-12);
u2 = u(2:2:end);
z  = sqrt(-2*log(u1)) .* cos(2*pi*u2);
z  = z(:);
end


%%
%  Response metrics

function m = metrics(r, c)
t = r.t; y = r.theta; tgt = c.setpoint;

% Everything about the step itself is measured BEFORE the disturbance arrives
if c.disturbance == 0
    pre = true(size(t));
else
    pre = t < c.tDist;
end
tp  = t(pre); yp = y(pre);

% a negative setpoint (turning left) measures exactly the same way a positive one does.
prog = yp/tgt;                  % 0 at the start, 1 at the target, >1 = overshoot

i10 = find(prog >= 0.10, 1, 'first');
i90 = find(prog >= 0.90, 1, 'first');
if isempty(i10) || isempty(i90), m.rise = NaN; else, m.rise = tp(i90) - tp(i10); end

m.overshoot = max(0, (max(prog) - 1)*100);

out = find(abs(yp - tgt) > 1.0, 1, 'last');
if isempty(out)
    m.settle = 0;               % never left the band in the first place
elseif tp(end) - tp(out) < 0.10
    m.settle = NaN;             % still moving when we ran out of run to look at
else
    m.settle = tp(out);
end

% Final error is measured at the very end, after any disturbance has been
% dealt with (or not dealt with, if you have no integral term).
m.finalErr = tgt - mean(y(t > t(end) - 0.20));

m.chatter  = sqrt(mean(diff(r.uTicks).^2));
m.saturated = 100*mean(abs(r.u) >= c.uMax - 1e-9);

m.pass = m.overshoot <= c.maxOvershoot && ...
         ~isnan(m.settle) && m.settle <= c.maxSettle && ...
         abs(m.finalErr) <= c.maxFinalErr;
end


%% ========================================================================
%  Console report
%  =======================================================================
function report(kp, ki, kd, c, m)
fprintf('\n  Kp = %-8.4g Ki = %-8.4g Kd = %-8.5g', kp, ki, kd);
fprintf('  (setpoint %g deg, disturbance %g)\n', c.setpoint, c.disturbance);
fprintf('  ---------------------------------------------------------------\n');
fprintf('    rise time (10%%->90%%) %11s\n', val(m.rise, 3, 's'));
fprintf('    overshoot            %11s   %s\n', val(m.overshoot, 1, '%'), ...
        mark(m.overshoot <= c.maxOvershoot, c.maxOvershoot, '%'));
fprintf('    settling (+-1 deg)   %11s   %s\n', val(m.settle, 3, 's'), ...
        mark(~isnan(m.settle) && m.settle <= c.maxSettle, c.maxSettle, 's'));
fprintf('    final error          %11s   %s\n', sval(m.finalErr, 2, 'deg'), ...
        mark(abs(m.finalErr) <= c.maxFinalErr, c.maxFinalErr, 'deg'));
fprintf('  ---------------------------------------------------------------\n');
fprintf('    command chatter      %11s   how twitchy the motor command is\n', ...
        val(m.chatter, 3, ''));
fprintf('    motor maxed out      %11s   of the run\n', val(m.saturated, 1, '%'));
if m.pass
    fprintf('\n    ==> TARGET MET\n\n');
else
    fprintf('\n    ==> not there yet\n\n');
end
end

function s = val(v, dec, unit)
if isnan(v)
    s = 'never';
else
    s = sprintf(sprintf('%%.%df %%s', dec), v, unit);
end
end

function s = sval(v, dec, unit)   % same, but always shows the sign
if isnan(v)
    s = 'never';
else
    s = sprintf(sprintf('%%+.%df %%s', dec), v, unit);
end
end

function s = mark(ok, lim, unit)
if ok
    s = sprintf('ok    (limit %g %s)', lim, unit);
else
    s = sprintf('FAIL  (limit %g %s)', lim, unit);
end
end


%% ========================================================================
%  The main plot
%  =======================================================================
function plot_run(r, c, m, kp, ki, kd)
figure('Color','w','Name','PID playground','Position',[100 100 900 640]);
tgt = c.setpoint;

% A fixed, generous view so the annotations always land somewhere sensible.
yTop = max(tgt*1.55, max(r.theta)*1.12);
yBot = min(-0.10*tgt, min(r.theta)*1.10);

% ---- yaw ---------------------------------------------------------------
ax1 = subplot(3,1,[1 2]); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'off');

fill([0 c.tEnd c.tEnd 0], [tgt-1 tgt-1 tgt+1 tgt+1], [0.16 0.47 0.84], ...
     'FaceAlpha', 0.10, 'EdgeColor','none');
plot(ax1, [0 c.tEnd], [tgt tgt], '--', 'Color',[.54 .54 .52], 'LineWidth',1.4);
stairs(ax1, r.t, r.meas, '-', 'Color',[0.11 0.69 0.48], 'LineWidth',0.8);
plot(ax1, r.t, r.theta, '-', 'Color',[0.16 0.47 0.84], 'LineWidth',2.2);

if c.disturbance ~= 0
    plot(ax1, [c.tDist c.tDist], [yBot yTop], ':', ...
         'Color',[0.89 0.29 0.28], 'LineWidth',1.3);
    text(ax1, c.tDist + 0.03, yTop - 0.07*(yTop-yBot), ...
         sprintf('disturbance %+.2f', c.disturbance), ...
         'Color',[0.89 0.29 0.28], 'FontWeight','bold', 'FontSize',9);
end

% mark the peak and the settling point (same window the metrics use)
if c.disturbance == 0
    pre = true(size(r.t));
else
    pre = r.t < c.tDist;
end
[pk, ipk] = max(r.theta(pre)); tpre = r.t(pre);
if pk > tgt + 0.05
    plot(ax1, tpre(ipk), pk, 'v', 'MarkerSize',7, 'MarkerFaceColor','k', 'MarkerEdgeColor','k');
    text(ax1, tpre(ipk)+0.04, pk + 0.06*(yTop-yBot), ...
         sprintf('overshoot %.1f%%', m.overshoot), 'FontSize',9, 'FontWeight','bold');
end
if ~isnan(m.settle)
    ySet = interp1(r.t, r.theta, m.settle);
    plot(ax1, m.settle, ySet, 'o', 'MarkerSize',7, ...
         'MarkerFaceColor','k','MarkerEdgeColor','k');
    plot(ax1, [m.settle m.settle], [tgt-1 tgt-0.16*(yTop-yBot)], ':', ...
         'Color',[0.32 0.32 0.31], 'LineWidth',1.0);
    text(ax1, m.settle+0.04, tgt-0.20*(yTop-yBot), ...
         sprintf('settles %.3f s', m.settle), 'FontSize',9, 'FontWeight','bold');
end

ylim(ax1, [yBot yTop]);
ylabel(ax1,'yaw (deg)');
title(ax1, sprintf('Kp = %.4g    Ki = %.4g    Kd = %.5g', kp, ki, kd), ...
      'FontWeight','bold');
legend(ax1, {'\pm1\circ settling band','setpoint','what the IMU reports','true yaw'}, ...
       'Location','southeast','Box','off');
xlim(ax1,[0 c.tEnd]);

% ---- command -----------------------------------------------------------
ax2 = subplot(3,1,3); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'off');
plot(ax2, [0 c.tEnd], [ c.uMax  c.uMax], ':', 'Color',[0.89 0.29 0.28],'LineWidth',1.3);
plot(ax2, [0 c.tEnd], [-c.uMax -c.uMax], ':', 'Color',[0.89 0.29 0.28],'LineWidth',1.3);
plot(ax2, r.t, r.u, '-', 'Color',[0.92 0.41 0.20], 'LineWidth',1.4);
text(ax2, 0.02, c.uMax*1.28, 'motor limit', 'Color',[0.89 0.29 0.28], ...
     'FontSize',9, 'FontWeight','bold');
xlabel(ax2,'time (s)'); ylabel(ax2,'command u');
ylim(ax2, [-c.uMax*1.6 c.uMax*1.6]); xlim(ax2,[0 c.tEnd]);
linkaxes([ax1 ax2],'x');
end


%% 
%  Sweep mode: overlay several values of one gain

function sweep(kp, ki, kd, c, gainName, values)
figure('Color','w','Name','PID sweep','Position',[100 100 900 640]);
ax1 = subplot(3,1,[1 2]); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'off');
ax2 = subplot(3,1,3);     hold(ax2,'on'); grid(ax2,'on'); box(ax2,'off');

cols = [0.16 0.47 0.84; 0.92 0.41 0.20; 0.11 0.69 0.48; 0.93 0.63 0.00; 0.29 0.23 0.65];
names = cell(1, numel(values));

plot(ax1, [0 c.tEnd], [c.setpoint c.setpoint], '--', ...
     'Color',[.54 .54 .52], 'LineWidth',1.4);

fprintf('\n  Sweeping %s.  Setpoint %g deg, disturbance %g.\n', ...
        gainName, c.setpoint, c.disturbance);
held = {sprintf('Kp %g', kp), sprintf('Ki %g', ki), sprintf('Kd %g', kd)};
switch gainName
    case 'kp', held(1) = [];
    case 'ki', held(2) = [];
    case 'kd', held(3) = [];
end
fprintf('  Held fixed (from the top of the file): %s, %s\n', held{1}, held{2});
fprintf('  ---------------------------------------------------------------------\n');
fprintf('  %-10s %10s %12s %12s %12s %10s\n', gainName, 'rise', 'overshoot', 'settling', 'final err', 'chatter');

for k = 1:numel(values)
    a = kp; b = ki; d = kd;
    switch gainName
        case 'kp', a = values(k);
        case 'ki', b = values(k);
        case 'kd', d = values(k);
        otherwise, error('pid_playground:gain','gain must be ''kp'', ''ki'' or ''kd''.');
    end
    r = simulate(a, b, d, c);
    m = metrics(r, c);
    col = cols(mod(k-1, size(cols,1))+1, :);
    plot(ax1, r.t, r.theta, '-', 'Color', col, 'LineWidth', 2);
    plot(ax2, r.t, r.u,     '-', 'Color', col, 'LineWidth', 1.2);
    names{k} = sprintf('%s = %g', gainName, values(k));

    if isnan(m.settle), st = '    never  '; else, st = sprintf('%9.3f s', m.settle); end
    fprintf('  %-10g %8.3f s %10.1f %% %s %10.2f deg %10.3f\n', ...
            values(k), m.rise, m.overshoot, st, m.finalErr, m.chatter);
end
fprintf('\n');

ylabel(ax1,'yaw (deg)'); xlim(ax1,[0 c.tEnd]);
title(ax1, sprintf('Sweeping %s', gainName), 'FontWeight','bold');
legend(ax1, [{'setpoint'} names], 'Location','southeast','Box','off');
plot(ax2, [0 c.tEnd], [ c.uMax  c.uMax], ':', 'Color',[0.89 0.29 0.28]);
plot(ax2, [0 c.tEnd], [-c.uMax -c.uMax], ':', 'Color',[0.89 0.29 0.28]);
xlabel(ax2,'time (s)'); ylabel(ax2,'command u'); xlim(ax2,[0 c.tEnd]);
ylim(ax2, [-c.uMax*1.6 c.uMax*1.6]);
linkaxes([ax1 ax2],'x');
end


%% 
%  IMU mode: what the controller sees

function show_imu(c)
% Always the same picture regardless of what the student has left at the top
% of the file, so it matches the figure in the handout.
c.setpoint = 20; c.disturbance = 0;
r = simulate(0.08, 0, 0.004, c);

figure('Color','w','Name','IMU measurement','Position',[100 100 1000 420]);

ax1 = subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'off');
plot(ax1, r.t, r.theta, '-', 'Color',[0.16 0.47 0.84], 'LineWidth',2.2);
stairs(ax1, r.t, r.meas, '-', 'Color',[0.11 0.69 0.48], 'LineWidth',1.0);
xlabel(ax1,'time (s)'); ylabel(ax1,'yaw (deg)'); xlim(ax1,[0 1.0]);
title(ax1,'Full move','FontWeight','bold');
legend(ax1,{'true yaw','what the controller sees'}, ...
       'Location','southeast','Box','off');

ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'off');
plot(ax2, r.t, r.theta, '-', 'Color',[0.16 0.47 0.84], 'LineWidth',2.4);
stairs(ax2, r.t, r.meas, '-', 'Color',[0.11 0.69 0.48], 'LineWidth',1.8);
xlabel(ax2,'time (s)'); ylabel(ax2,'yaw (deg)');
xlim(ax2,[0.30 0.42]); ylim(ax2,[19.72 20.62]);
title(ax2, sprintf('120 ms of the process, zoomed: a new reading every %g ms', ...
                   1000/c.fsImu), 'FontWeight','bold');

fprintf('\n  The simulated IMU, matching the BNO055 on our robots:\n');
fprintf('    update rate   %g Hz      (so the reading is up to %g ms stale)\n', ...
        c.fsImu, 1000/c.fsImu);
fprintf('    resolution    %.4f deg  (16 counts per degree)\n', c.lsb);
fprintf('    noise         %.3f deg  (1 sigma)\n', c.noise);
fprintf('    delay         %g ms      (I2C + the sensor''s internal filter)\n\n', ...
        c.delay*1000);
fprintf('  Our controller never sees the blue line. Every decision it makes\n');
fprintf('  is based on the green one.\n\n');
end
