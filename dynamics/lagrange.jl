module lagrange

using SymEngine

# Calculates the jacobian of vector-fucntion w.r.t a vector
function jacobian(f,x)
    m = length(f)
    n = length(x)

    jac = [diff(f[i],x[j]) for i in 1:m, j in 1:n]

    return jac
end

# Calculates the time derivative of a vector
function ddt(r, state)
    jacobian(r, [state[1];state[2]])*[state[2];state[3]]
end

# Calculates generalized torque
function M2Q(M,w,dq)
    jacobian(w,dq)'*M
end

end
