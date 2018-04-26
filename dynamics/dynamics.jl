#!/usr/bin/env julia
module dynamics

using DifferentialEquations

lorenz = @ode_def Lorenz begin
    dx = σ*(y-x)
    dy = ρ*x-y-x*z
    dz = x*y-β*z
end σ ρ β

d_integrator = @ode_def d_int begin
    dx = v
    dv = u/m
end m

d_integrator_wrong = @ode_def d_int_wrong begin
    m = m - 0.5
    dx = v
    dv = u/m
end m

function manipulator(du, u, p, t)
    # Unpack the parameters into standar manipulator eqn form:
    # M(θ)̈θ + D(̇θ) + C(θ, ̇θ) + G(θ) = τ
    l1 = p[1]; l2 = p[2];
    m1 = p[3]; m2 = p[4];
    I1 = p[5]; I2 = p[6];
    b1 = p[7]; b2 = p[7];
    g  = p[8]

    M11 = (m1*l1^2)/4 + m2*(l1^2 + l1*l2^3*cos(u[2])/4) + I1 + I2
    M12 = m2*l2^2/4 + m2*l1*l2*cos(u[2])/2 + I2
    M22 = m2*l2^2/4 + I2

    C11 = -m2*l1*l2*u[5]*sin(u[2])
    C12 = -m2*l1*l2*u[5]*sin(u[2])/2
    C21 = m2*l1*l2*u[4]*sin(u[2])/2
    C22 = 0.0

    G1  = (g/2)*(l1*(2*m1+m2)*cos(u[1]) + m2*l2*cos(u[1] + u[2]))
    G2  = m2*g*l2*cos(u[1] + u[2])/2

    M = [M11 M12; M12 M22]
    D = [b1; b2]
    C = [C11 C12; C21 C22]
    G = [G1     ; G2     ]
    # τ = p[8]

    # Generate control law
    τ = [0.0;0.0;0.0]

    # Invert the mass matrix
    M_inv = inv(M)

    # Manipulator Dynamics
    du[1] = u[4]
    du[2] = u[5]
    du[3] = u[6]
    du[4] = dot(M_inv[1,:], τ[1] - D[1]*u[4] - C[1,:] - G[1])
    #     du[4] = dot(M_inv[1,:], -C[1,:] - G[1])

    du[5] = dot(M_inv[2,:], τ[2] - D[2]*u[5] - C[2,:] - G[2])
    #     du[5] = dot(M_inv[2,:], -C[2,:] - G[2])

    du[6] = τ[3]
end



end
