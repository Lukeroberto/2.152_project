#!/usr/bin/env julia
module simulation

using DifferentialEquations


function simulate(f, u0, tspan, p)
    prob = ODEProblem(f, u0, tspan, p)
    return solve(prob)
end

function rect_animate()
    for i = 1:50
        plot(rectangle(0.1*i,0.1*i,2,2), opacity=1, xlims=(0,10),ylims=(0,10),c=:black)
        sleep(0.1)
    end
end

end
