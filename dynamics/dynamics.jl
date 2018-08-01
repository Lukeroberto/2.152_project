#!/usr/bin/env julia
module dynamics

function manipulator_load(du, u, p, t)
    m1 = p[1]; l1 = p[2];
    me = p[3]; de = p[4];
    I1 = p[5]; lc1 = p[6];
    Ie = p[7]; lce = p[8];
    b1 = p[9]; b2 = p[10];
    g = p[11];

    q1  = u[1]
    q2  = u[2]
    dq1 = u[3]
    dq2 = u[4]

    control_law = p[12](t,u);

    a1 = I1 + m1*lc1^2 + Ie + me*lce^2 + me*l1^2
    a2 = Ie + me*lce^2
    a3 = me*l1*lce*cos(de)
    a4 = me*l1*lce*sin(de)
    h  = a3*sin(q2) - a4*cos(q2)

    M11 = a1 + 2*a3*cos(q2) + 2*a4*sin(q2)
    M12 = a2 + a3*cos(q2) + a4*sin(q2)
    M22 = a2

    C11 = -h*dq2
    C12 = -h*(dq1 + dq2)
    C21 = h*dq1
    C22 = 0.

    G1  = 0. # (g/2)*(l1*(2*m1+m2)*cos(u[1]) + m2*l2*cos(u[1] + u[2]))
    G2  = 0. # m2*g*l2*cos(u[1] + u[2])/2

    M = [M11 M12; M12 M22]
    Minv = inv(M);

    D = [b1; b2]
    C = [C11 C12; C21 C22]
    G = [G1     ; G2     ]

    # println(u)
    # println(control_law)
    # println("\n")
    ddu = Minv*(control_law - C*[dq1;dq2]);

    du[1] = dq1
    du[2] = dq2
    du[3] = ddu[1]
    du[4] = ddu[2]
end

function manipulator_adaptive(du, u, p, t)
    m1 = p[1]; l1 = p[2];
    me = p[3]; de = p[4];
    I1 = p[5]; lc1 = p[6];
    Ie = p[7]; lce = p[8];
    b1 = p[9]; b2 = p[10];
    g = p[11];

    q1  = u[1]
    q2  = u[2]
    a   = u[3:6]
    dq1 = u[7]
    dq2 = u[8]

    control_law = p[12](t,u)
    adaptation_law = p[13](t, u)

    a1 = I1 + m1*lc1^2 + Ie + me*lce^2 + me*l1^2
    a2 = Ie + me*lce^2
    a3 = me*l1*lce*cos(de)
    a4 = me*l1*lce*sin(de)
    h  = a3*sin(q2) - a4*cos(q2)

    M11 = a1 + 2*a3*cos(q2) + 2*a4*sin(q2)
    M12 = a2 + a3*cos(q2) + a4*sin(q2)
    M22 = a2

    C11 = -h*dq2
    C12 = -h*(dq1 + dq2)
    C21 = h*dq1
    C22 = 0.

    G1  = 0. # (g/2)*(l1*(2*m1+m2)*cos(u[1]) + m2*l2*cos(u[1] + u[2]))
    G2  = 0. # m2*g*l2*cos(u[1] + u[2])/2

    M = [M11 M12; M12 M22]
    Minv = inv(M);

    D = [b1; b2]
    C = [C11 C12; C21 C22]
    G = [G1     ; G2     ]

    # println(u)
    # println(control_law)
    # println("\n")
    ddu = Minv*(control_law - C*[dq1;dq2]);

    # println(ddu)
    # println(adaptation_law)
    du[1] = dq1
    du[2] = dq2
    du[3:6] = adaptation_law
    du[7] = ddu[1]
    du[8] = ddu[2]
end

function manipulator_composite_adaptive(du, u, p, t)
    m1 = p[1]; l1 = p[2];
    me = p[3]; de = p[4];
    I1 = p[5]; lc1 = p[6];
    Ie = p[7]; lce = p[8];
    b1 = p[9]; b2 = p[10];
    g = p[11];

    q1  = u[1]
    q2  = u[2]
    a   = u[3:6]
    dq1 = u[7]
    dq2 = u[8]

    control_law = p[12](t,u)
    adaptation_law = p[13](t, u)
    gain_update = p[14](t,u)

    a1 = I1 + m1*lc1^2 + Ie + me*lce^2 + me*l1^2
    a2 = Ie + me*lce^2
    a3 = me*l1*lce*cos(de)
    a4 = me*l1*lce*sin(de)
    h  = a3*sin(q2) - a4*cos(q2)

    M11 = a1 + 2*a3*cos(q2) + 2*a4*sin(q2)
    M12 = a2 + a3*cos(q2) + a4*sin(q2)
    M22 = a2

    C11 = -h*dq2
    C12 = -h*(dq1 + dq2)
    C21 = h*dq1
    C22 = 0.

    G1  = 0. # (g/2)*(l1*(2*m1+m2)*cos(u[1]) + m2*l2*cos(u[1] + u[2]))
    G2  = 0. # m2*g*l2*cos(u[1] + u[2])/2

    M = [M11 M12; M12 M22]
    Minv = inv(M);

    D = [b1; b2]
    C = [C11 C12; C21 C22]
    G = [G1     ; G2     ]

    # println(u)
    # println(control_law)
    # println("\n")
    ddu = Minv*(control_law - C*[dq1;dq2]);

    du[1] = dq1
    du[2] = dq2
    du[3:6] = adaptation_law
    du[7:10] = gain_update
    du[11] = ddu[1]
    du[12] = ddu[2]
end

end
