module adaptive_control

function desired_states(t)
    q1d = (pi/6)*(1 - cos(2*pi*t))
    q2d = (pi/4)*(1 - cos(2*pi*t))
    dq1d = 2*pi*(pi/6)*sin(2*pi*t)
    dq2d = 2*pi*(pi/4)*sin(2*pi*t)
    ddq1d = (2*pi)^2*(pi/6)*cos(2*pi*t)
    ddq2d = (2*pi)^2*(pi/4)*cos(2*pi*t)

    [q1d;q2d;dq1d;dq2d;ddq1d;ddq2d]
end


function Y(t, state, lambda)
    q1  = state[1]
    q2  = state[2]
    a_hat   = state[3:6]
    dq1 = state[7]
    dq2 = state[8]

    q1d, q2d, dq1d, dq2d, ddq1d, ddq2d = desired_states(t)

    dqr  = [dq1d;dq2d] - lambda*(state[1:2] - [q1d;q2d])
    ddqr = [ddq1d;ddq2d] - lambda*(state[7:8] - [dq1d;dq2d])

    Y11 = ddqr[1]
    Y12 = ddqr[2]
    Y13 = (2*ddqr[1] + ddqr[2])*cos(q2) - (dq2*dqr[1] + dq1*dqr[2] + dq2*dqr[2])*sin(q2)
    Y14 = (2*ddqr[1] + ddqr[2])*sin(q2) + (dq2*dqr[1] + dq1*dqr[2] + dq2*dqr[2])*cos(q2)
    Y21 = 0
    Y22 = ddqr[1] + ddqr[2]
    Y23 = ddqr[1]*cos(q2) + dq1*dqr[1]*sin(q2)
    Y24 = ddqr[1]*sin(q2) - dq1*dqr[1]*cos(q2)

    [Y11 Y12 Y13 Y14; Y21 Y22 Y23 Y24]
end

function W(t, state)

end

function forgetting_factor(t, P,lambda_0, k_0)
    lambda_0*(1 - norm(P)/k_0)
end

function gain_update(t, state, P, W)
    lam = forgetting_factor(P, lambda_0, k_0)
    lam*P - P*W'*W*P
end

function adaptive_control_law(t, state, lambda, Kd)
    q1  = state[1]
    q2  = state[2]
    a_hat   = state[3:6]
    dq1 = state[7]
    dq2 = state[8]

    q1d, q2d, dq1d, dq2d, ddq1d, ddq2d = desired_states(t)

    s(t,state) = ([dq1;dq2] - [dq1d;dq2d]) + lambda*([q1;q2] - [q1d;q2d])
    Y(t,state, lambda)*a_hat - Kd*s(t,state)
end

function adaptation_law_arm(t, state, lambda, gamma)
    q1  = state[1]
    q2  = state[2]
    a_hat   = state[3:6]
    dq1 = state[7]
    dq2 = state[8]

    q1d, q2d, dq1d, dq2d, ddq1d, ddq2d = desired_states(t)

    s(t,state) = ([dq1;dq2] - [dq1d;dq2d]) + lambda*([q1;q2] - [q1d;q2d])
    -gamma*Y(t, state, lambda)'*s(t, state)
end


function composite_adaption_law(t, state, lambda_0, k_0)
    q1  = state[1]
    q2  = state[2]
    a_hat = state[3:6]
    P_t  = state[7:10]
    dq1 = state[11]
    dq2 = state[12]

    q1d, q2d, dq1d, dq2d, ddq1d, ddq2d = desired_states(t)
    P = [P_t[1:2]';P_t[3:4]']

    s(t,state) = ([dq1;dq2] - [dq1d;dq2d]) + lambda*([q1;q2] - [q1d;q2d])


    -P*(Y(t, state, lambda)'*s(t, state) + W'*(y - W*a_hat))

end

end
