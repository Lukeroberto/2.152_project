#!/usr/bin/env julia
module simulation

using DifferentialEquations


function simulate(f, u0, tspan, p)
    prob = ODEProblem(f, u0, tspan, p)
    return solve(prob)
end

rectangle(w, h, x, y) = Shape(x + [0,w,w,0], y + [0,0,h,h])
plt = plot()
plot!(xlims=(0,10))
plot!(ylims=(0,10))

function rect_animate()
    for i = 1:50
        plot(rectangle(0.1*i,0.1*i,2,2), opacity=1, xlims=(0,10),ylims=(0,10),c=:black)
        sleep(0.1)
    end
end

rot(x,x0, θ) = [(x[1])*cos(θ) + (x[2])*sin(θ), -(x[1])*sin(θ) + (x[2])*cos(θ)]
square_edges(l,x,y) = [[l/2,l/2] [l/2,-l/2] [-l/2,l/2] [-l/2,-l/2]]

function rotated_square(x,y,θ,l)
    square = square_edges(l,x,y)
    p1 = rot(square[:,1],[x,y], θ)
    p2 = rot(square[:,2],[x,y], θ)
    p3 = rot(square[:,3],[x,y], θ)
    p4 = rot(square[:,4],[x,y], θ)
    return Shape([p1[1], p2[1], p3[1], p4[1]],[p1[2], p2[2], p3[2], p4[2]])
end

# for i=0.0:0.01:2π
#     plot(rotated_square(5,5,i,2), opacity=1, xlims=(0,10), ylims=(0,10))
#     sleep(0.5)
# end

export simulate

end
