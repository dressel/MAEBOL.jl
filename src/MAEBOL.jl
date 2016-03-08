module MAEBOL

import StatsBase: entropy, WeightVec, sample
using Distributions: VonMises, Normal, cdf
using PyPlot: imshow, plot, xlabel, ylabel, contour, figure, pause, hold
using Cubature

export SearchDomain, initial_belief, update_belief!, transition!
export Vehicle, VehicleSet
export O, observe, true_bearing
export fisher, EID
export plot_world, plot_eid, plot_eid2, plot_eid3, meshgrid, plot_sim
export h_ij, hk2, phik, fk
export ErgodicManager
export get_action, RandPolicy
export Simulation

typealias Obs Int64
typealias Observation Obs
typealias ObsSet Vector{Obs}
typealias ActionSet Vector{NTuple{2, Float64}}
typealias Belief Matrix{Float64}

include("vehicleset.jl")
include("searchdomain.jl")
include("observations.jl")
include("ergodicity.jl")
include("eid.jl")
include("policy.jl")
include("simulation.jl")
include("plotting.jl")

# TODO: add checks for staying within
"""
`transition(m, x, a)`

Returns vehicle state after taking action `a`. Not currently working.
"""
function transition!(m::SearchDomain, X::VehicleSet, A::ActionSet)
	for i = 1:length(X)
		X[i].x += A[i][1]
		X[i].y += A[i][2]
	end
end


end # module
