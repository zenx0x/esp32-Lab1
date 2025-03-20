%internet of things challenge 1
%nember XU XUELI :11075199
%       SUN YILIN :11072044
% Optimizing Sink Position in a Wireless Sensor Network


sensors = [
    1, 2;
    10, 3;
    4, 8;
    15, 7;
    6, 1;
    9, 12;
    14, 4;
    3, 10;
    7, 7;
    12, 14
];
% Initial energy per sensor
Eb = 5 * 1e-3;
% Energy for the TX/RX circuitry
Ec = 50 * 1e-9;
% Energy for transmission
k = 1 * 1e-9;
b = 2000;
% Transmission interval time (minutes)
trans_interval = 10;

% Calculate the system lifetime and sensor information with a fixed sink position
sink_fixed = [20, 20];
[fixed_lifetime, sensor_info] = system_lifetime_fixed_sink(sensors, sink_fixed, Eb, Ec, k, b, trans_interval);
fprintf('System lifetime when Sink is at (%d, %d): %.2f cycles, corresponding time: %.2f hours\n', sink_fixed(1), sink_fixed(2), fixed_lifetime, fixed_lifetime * trans_interval / 60);

% Find the optimal sink position and the corresponding system lifetime
[best_sink, best_lifetime] = simulated_annealing(sensors, Eb, Ec, k, b);
fprintf('Optimal Sink position: (%.2f, %.2f)\n', best_sink(1), best_sink(2));
fprintf('System lifetime at the optimal position: %.2f cycles, corresponding time: %.2f hours\n', best_lifetime, best_lifetime * trans_interval / 60);

% Compare the system lifetimes between the fixed and dynamic sink positions
fprintf('\nComparison between fixed and dynamic sink positions:\n');
fprintf('System lifetime with fixed Sink position (%d, %d): %.2f cycles, corresponding time: %.2f hours\n', sink_fixed(1), sink_fixed(2), fixed_lifetime, fixed_lifetime * trans_interval / 60);
fprintf('System lifetime at the optimal dynamic Sink position: %.2f cycles, corresponding time: %.2f hours\n', best_lifetime, best_lifetime * trans_interval / 60);

function [lifetime, sensor_info] = system_lifetime_fixed_sink(sensors, sink, Eb, Ec, k, b, trans_interval)
    num_sensors = size(sensors, 1);
    lifetimes = zeros(num_sensors, 1);
    sensor_info = zeros(num_sensors, 5);

    fprintf('Sensor information:\n');
    fprintf('Sensor ID\tPosition (x,y)\tDistance\tTransmission energy (J)\t\tNumber of transmissions\t\tTransmission time (hours)\n');
    for i = 1:num_sensors
        d = norm(sensors(i, :) - sink);
        % Calculate the energy consumption per transmission
        E_per_trans = (Ec + k * d^2) * b;
        % Calculate the number of transmissions
        if E_per_trans > 0
            lifetimes(i) = Eb / E_per_trans;
        else
            lifetimes(i) = Inf;
        end
        trans_time = lifetimes(i) * trans_interval / 60;

        sensor_info(i, 1) = i;
        sensor_info(i, 2) = d;
        sensor_info(i, 3) = E_per_trans;
        sensor_info(i, 4) = lifetimes(i);
        sensor_info(i, 5) = trans_time;

        fprintf('%d\t\t\t(%d, %d)\t\t\t%.2f\t\t%.2e\t\t\t\t\t%.2e\t\t\t\t\t%.2e\n', i, sensors(i, 1), sensors(i, 2), d, E_per_trans, lifetimes(i), trans_time);
    end
    % The system lifetime is determined by the minimum number of transmissions
    lifetime = min(lifetimes);
end

% Use simulated annealing to find the optimal position
function [best_sink, best_lifetime] = simulated_annealing(sensors, Eb, Ec, k, b)
    T = 100;
    alpha = 0.97;
    num_iterations = 2000;

    current_sink = [10, 10];
    current_max_energy = inf;

    for iter = 1:num_iterations
        new_sink = current_sink + 2 * (rand(1, 2) - 0.5);
        new_sink = max([0, 0], min([20, 20], new_sink));

        % Calculate the maximum energy consumption of all sensors at the new position
        num_sensors = size(sensors, 1);
        energies = zeros(num_sensors, 1);
        for i = 1:num_sensors
            d = norm(sensors(i, :) - new_sink);
            energies(i) = (Ec + k * d^2) * b;
        end
        new_max_energy = max(energies);

        % Update the sink position
        if new_max_energy < current_max_energy
            current_sink = new_sink;
            current_max_energy = new_max_energy;
        else
            p = exp((current_max_energy - new_max_energy) / T);
            if rand() < p
                current_sink = new_sink;
                current_max_energy = new_max_energy;
            end
        end

        T = T * alpha;
    end

    % Calculate the system lifetime at the optimal position
    num_sensors = size(sensors, 1);
    lifetimes = zeros(num_sensors, 1);
    for i = 1:num_sensors
        d = norm(sensors(i, :) - current_sink);
        E_per_trans = (Ec + k * d^2) * b;
        if E_per_trans > 0
            lifetimes(i) = Eb / E_per_trans;
        else
            lifetimes(i) = Inf;
        end
    end
    best_lifetime = min(lifetimes);
    best_sink = current_sink;
end    