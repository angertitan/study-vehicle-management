% init
dt = 0.01;
K_p = 2;
D = 0.2;
w_0 = 1;
sim_length = (30* (1/w_0))/dt;

vec__X_e = ones(1, sim_length);

vec__X_1 = zeros(1, sim_length);
vec__X_1(1) = 0;

vec__X_2 = zeros(1, sim_length);
vec__X_2(1) = 0;

vec__t = zeros(1, sim_length);

% simulation X_2
for k=1: sim_length
    vec__X_1(k+1) = vec__X_1(k) + dt * (vec__X_2(k));
    vec__X_2(k+1) = vec__X_2(k) + dt * (-(w_0)^2*vec__X_1(k)-2*D*w_0*vec__X_2(k)+K_p*w_0^2*vec__X_e(k));
    vec__t(k) = (k - 1) * dt;
end

plot(vec__t, vec__X_1(1:end-1));
hold on
plot(vec__t, vec__X_2(1:end-1));
hold off