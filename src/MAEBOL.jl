module MAEBOL

import StatsBase: entropy, WeightVec, sample
using Distributions: VonMises, Normal, cdf
using PyPlot: imshow, plot, xlabel, ylabel, contour
using Cubature

export SearchDomain, initial_belief, update_belief
export Vehicle, VehicleSet
export O, observe, true_bearing
export fisher, EID
export plot_world, plot_eid, plot_eid2, plot_eid3, meshgrid
export h_ij, hk2, phik, fk
export ErgodicManager

typealias Obs Int64
typealias Observation Obs
typealias Belief Matrix{Float64}

include("vehicleset.jl")
include("searchdomain.jl")
include("observations.jl")
include("ergodicity.jl")
include("eid.jl")
include("plotting.jl")

"""
`transition(m, x, a)`

Returns vehicle state after taking action `a`. `Not` currently working.
"""
function transition(m, x, a)
end



end # module
