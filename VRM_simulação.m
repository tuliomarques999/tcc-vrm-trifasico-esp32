clear;
close all;
clc;

% Parametros do sistema
Vin = 12;
Vref = 5;
L = 100e-6;
rL = 0.1;
Co = 470e-6;
rC = 0.05;
R = Vref / 6; % carga nominal (Iout = 6A)
Ts = 100e-6;

fprintf('=== VALIDACAO VRM — CONTROLADOR PI ===\n\n');
fprintf('Vin=%.0fV | Vref=%.0fV | L=%.0fuH | Co=%.0fuF | R=%.4f ohm\n\n', ...
    Vin, Vref, L*1e6, Co*1e6, R);

% Criterios do TCC
PM_req = 45;
GM_req = 6;
OS_req = 20;
Ts_req = 5;
ess_req = 1;

fases = [3 2 1];
cores = {'b', 'r', 'g'};
resultado = struct();

fprintf('%-5s %-9s %-9s %-11s %-11s %-9s %-7s %-7s\n', ...
    'N', 'Kp', 'Ki', 'GM(dB)', 'PM(deg)', 'OS(%%)', 'ess(%%)', 'Ts(ms)');
fprintf('%s\n', repmat('-', 1, 75));

for k = 1:3
    Nf = fases(k);

    % Planta G(s) - Buck multifasico
    G0 = Nf * Vin * R / (rL + Nf*R);
    Leq = L / Nf;
    RT = R + rC;
    wn = sqrt(1 / (Leq * Co));
    zeta = (rL/Nf + rC) / (2 * Leq * wn);

    num_G = G0 * [rC*Co 1];
    den_G = [Leq*Co, (rL/Nf*Co + Co*rC), 1];
    Gs = tf(num_G, den_G);

    % Projeto do PI
    wc = 0.30 * wn;
    Ti = 8 / wc;

    C_unit = tf([Ti 1], [Ti 0]);
    mag_GC = abs(evalfr(C_unit * Gs, 1j*wc));
    Kp = 1 / mag_GC;
    Ki = Kp / Ti;

    C_pi = tf(Kp * [Ti 1], [Ti 0]);
    C_piz = c2d(C_pi, Ts, 'tustin');

    L_ol = C_pi * Gs;
    Tcl = feedback(L_ol, 1);

    [Gm, Pm] = margin(L_ol);
    GMdB = 20 * log10(Gm);

    % Resposta ao degrau
    t = linspace(0, 0.025, 3000);
    y = step(Vref * Tcl, t);

    yss = mean(y(end-100:end));
    ess = abs(Vref - yss) / Vref * 100;
    info = stepinfo(y, t, Vref, 'SettlingTimeThreshold', 0.02);
    OS = max(0, info.Overshoot);
    Ts_ms = info.SettlingTime * 1e3;

    ok_GM = GMdB >= GM_req;
    ok_PM = Pm >= PM_req;
    ok_OS = OS <= OS_req;
    ok_ess = ess <= ess_req;
    ok_Ts = Ts_ms <= Ts_req;

    stGM = s_ok(ok_GM);
    stPM = s_ok(ok_PM);
    stOS = s_ok(ok_OS);
    stE = s_ok(ok_ess);
    stTs = s_ok(ok_Ts);

    fprintf('N=%-3d Kp=%-7.4f Ki=%-7.2f GM=%-5.1f%s PM=%-5.1f%s OS=%-5.1f%s ess=%-5.2f%s Ts=%-5.2f%s\n', ...
        Nf, Kp, Ki, GMdB, stGM, Pm, stPM, OS, stOS, ess, stE, Ts_ms, stTs);

    [nz, dz] = tfdata(C_piz, 'v');

    resultado(k).Nf = Nf;
    resultado(k).Kp = Kp;
    resultado(k).Ki = Ki;
    resultado(k).Gs = Gs;
    resultado(k).C_pi = C_pi;
    resultado(k).C_piz = C_piz;
    resultado(k).Tcl = Tcl;
    resultado(k).L_ol = L_ol;
    resultado(k).GMdB = GMdB;
    resultado(k).Pm = Pm;
    resultado(k).OS = OS;
    resultado(k).ess = ess;
    resultado(k).Ts_ms = Ts_ms;
    resultado(k).nz = nz;
    resultado(k).dz = dz;
end

% Figura 1 - Resposta ao degrau
figure('Name', 'Degrau — Malha Fechada PI', 'Position', [50 50 1050 380]);

t = linspace(0, 0.025, 3000);

for k = 1:3
    subplot(1, 3, k);
    y = step(Vref * resultado(k).Tcl, t);
    plot(t*1e3, y, cores{k}, 'LineWidth', 1.8);
    hold on;
    yline(Vref, 'k--', 'Ref', 'LineWidth', 1.2);
    yline(Vref*1.02, 'k:', '+2%', 'LineWidth', 0.8);
    yline(Vref*0.98, 'k:', '-2%', 'LineWidth', 0.8);
    grid on;
    xlabel('Tempo (ms)');
    ylabel('Vout (V)');
    title(sprintf('N=%d   PM=%.1f°   Ts=%.2fms', ...
        resultado(k).Nf, resultado(k).Pm, resultado(k).Ts_ms));
    ylim([0 Vref*1.35]);
end

% Figura 2 - Bode malha aberta
w = logspace(2, 6, 1500);

figure('Name', 'Bode — Malha Aberta PI', 'Position', [50 480 850 500]);

subplot(2, 1, 1);
hold on;
grid on;
set(gca, 'XScale', 'log');
ylabel('Magnitude (dB)');
title('Bode — Malha Aberta  L(s) = C_{PI}(s) \cdot G(s)');
yline(0, 'k--', '0 dB', 'LineWidth', 1);

subplot(2, 1, 2);
hold on;
grid on;
set(gca, 'XScale', 'log');
ylabel('Fase (graus)');
xlabel('\omega  (rad/s)');
yline(-180, 'k--', '-180°', 'LineWidth', 1);

for k = 1:3
    H = squeeze(freqresp(resultado(k).L_ol, w));
    mag = 20 * log10(abs(H));
    ph = angle(H) * 180 / pi;

    subplot(2, 1, 1);
    semilogx(w, mag, cores{k}, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('N=%d', resultado(k).Nf));

    idx0 = find(diff(sign(mag)), 1);
    if ~isempty(idx0)
        semilogx(w(idx0), 0, 'o', 'Color', cores{k}, ...
            'MarkerFaceColor', cores{k}, 'MarkerSize', 7, ...
            'HandleVisibility', 'off');
    end

    subplot(2, 1, 2);
    semilogx(w, ph, cores{k}, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('N=%d', resultado(k).Nf));
end

subplot(2, 1, 1);
legend('Location', 'southwest');
xlim([w(1) w(end)]);

subplot(2, 1, 2);
legend('Location', 'southwest');
xlim([w(1) w(end)]);

% Figura 3 - Gain scheduling
figure('Name', 'Gain Scheduling — Falha', 'Position', [50 50 950 420]);
hold on;
grid on;

seg_t = {[0 0.015], [0.015 0.040], [0.040 0.070]};
Vfim = 0;

for k = 1:3
    ti = seg_t{k}(1);
    tf_seg = seg_t{k}(2);
    Tcl = resultado(k).Tcl;

    t_loc = linspace(0, tf_seg - ti, 1500);
    r_loc = Vref * ones(size(t_loc));
    y_loc = step(Tcl * Vref, t_loc);

    % conecta valor final do trecho anterior
    y_loc = y_loc - y_loc(1) + Vfim;
    Vfim = y_loc(end);

    plot((ti + t_loc)*1e3, y_loc, cores{k}, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('N=%d', resultado(k).Nf));

    if k < 3
        xline(tf_seg*1e3, 'k--', sprintf('Falha F%d', k), ...
            'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
end

yline(Vref, 'k-', 'Vref', 'LineWidth', 1.5);
yline(Vref*1.05, 'r:', '+5%', 'LineWidth', 0.9);
yline(Vref*0.95, 'r:', '-5%', 'LineWidth', 0.9);
xlabel('Tempo (ms)');
ylabel('Vout (V)');
title('Gain Scheduling com PI — Reconfiguração N:3\rightarrow2\rightarrow1');
legend('Location', 'southeast');
ylim([0 Vref*1.4]);

% Coeficientes C(z) para o ESP32
fprintf('\n--- Coeficientes C(z) para firmware ESP32 ---\n');
fprintf('%-5s %-10s %-10s %-10s %-10s\n', 'N', 'b0', 'b1', 'a0', 'a1');
fprintf('%s\n', repmat('-', 1, 50));

for k = 1:3
    fprintf('N=%-3d b=[%-7.5f %-7.5f]  a=[%-7.5f %-7.5f]\n', ...
        resultado(k).Nf, ...
        resultado(k).nz(1), resultado(k).nz(2), ...
        resultado(k).dz(1), resultado(k).dz(2));
end

fprintf('\n=== VALIDACAO CONCLUIDA ===\n');

function s = s_ok(cond)
    if cond
        s = '(OK)';
    else
        s = '(NOK)';
    end
end