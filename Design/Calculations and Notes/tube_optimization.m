% Define discrete values for OD and wall thickness
OD_values = [150 160 170 180 190 200]; % Example OD values (mm)
t_values = [5 6 7 8 9 10]; % Example wall thickness values (mm)

% Define material properties (example values)
densities = [7850 2700]; % Densities of materials (kg/m^3) 
youngs_moduli = [210e9 70e9]; % Young's moduli of materials (Pa) 

% Create a meshgrid to get all possible combinations
[OD, t] = meshgrid(OD_values, t_values);

% Calculate mass and stiffness for each combination
for i = 1:length(densities)
    mass(:,:,i) = pi * (OD.^2 - (OD - 2*t).^2) / 4 * length_tube * densities(i); % Calculate mass for each density
    stiffness(:,:,i) = (pi/64) * (OD.^4 - (OD - 2*t).^4) * youngs_moduli(i) / (length_tube^3); % Calculate stiffness for each Young's modulus
end

% Normalize mass and stiffness for multi-objective optimization
mass_normalized = (mass - min(mass(:))) / (max(mass(:)) - min(mass(:))); 
stiffness_normalized = (max(stiffness(:)) - stiffness) / (max(stiffness(:)) - min(stiffness(:))); 

% Define a combined objective function (e.g., weighted sum)
weight_mass = 0.5; % Adjust weights as needed
weight_stiffness = 1 - weight_mass;
objective_function = weight_mass * mass_normalized + weight_stiffness * stiffness_normalized;

% Find the index of the best design (minimizing the objective function) for each material
best_indices = zeros(size(densities));
for i = 1:length(densities)
    obj_func = objective_function(:,:,i);
    [~, best_indices(i)] = min(obj_func(:)); 
end

% Extract the optimal OD and wall thickness for each material
optimal_OD = zeros(size(densities));
optimal_t = zeros(size(densities));
for i = 1:length(densities)
    linear_index = best_indices(i); 
    [row, col] = ind2sub(size(OD), linear_index); 
    optimal_OD(i) = OD(row, col);
    optimal_t(i) = t(row, col);
end

% Display results
for i = 1:length(densities)
    fprintf('Optimal Design for Material %d:\n', i);
    fprintf('Density: %.2f kg/m^3\n', densities(i));
    fprintf('Youngs Modulus: %.2e Pa\n', youngs_moduli(i));
    fprintf('OD: %.2f mm\n', optimal_OD(i));
    fprintf('Wall Thickness: %.2f mm\n\n', optimal_t(i));
end

% Optional: Visualize the Pareto front (if desired)
% ... (Code for plotting mass vs. stiffness for all combinations and materials)
% ... (Identify Pareto optimal solutions)

% Notes:
% 1. Adjust `weight_mass` and `weight_stiffness` to prioritize mass or stiffness.
% 2. This code uses a simple weighted sum approach for multi-objective optimization. 
%    Consider more advanced techniques like Pareto optimization for more complex scenarios.
% 3. Replace the placeholder calculations for `mass` and `stiffness` with your actual formulas.
% 4. This code assumes a constant length_tube. Adjust accordingly.
% 5. For more accurate stiffness calculations, consider using finite element analysis (FEA).