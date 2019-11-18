% init
T_1 = 60;
dt = 1;
K_IS = 2;
sim_length = (10* T_1)/dt;
ROT_0 = 0;

vec__delta_r = ones(1, sim_length);
vec__ROT = zeros(1, sim_length);
vec__ROT(1) = ROT_0;
vec__t = zeros(1, sim_length);
% simulation

for k=1: sim_length
    vec__ROT(k+1) = vec__ROT(k) + dt * ((-1/T_1)*vec__ROT(k) + (K_IS/T_1)*vec__delta_r(k));
    vec__t(k) = (k - 1) * dt;
end

plot(vec__t, vec__ROT(1:end-1));
hold on
plot(vec__t, vec__delta_r);
hold off