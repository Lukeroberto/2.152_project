module lagrange

# Calculates the jacobian of vector-fucntion w.r.t a vector
function jacobian(f,x)
    m = length(f)
    n = length(x)

    jac = [diff(f[i],x[j]) for i in 1:m, j in 1:n]

    return jac
end

# Calculates the time derivative of a vector
function ddt(r, q,dq,ddq)
    jacobian(r, [q;dq])*[dq;ddq]
end

# Calculates generalized torque
function M2Q(M,w,dq)
    jacobian(w,dq)'*M
end

function F2Q(r,F,q)
    jacobian(r,q)'*F
end

end
