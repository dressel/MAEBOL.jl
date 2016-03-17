module MAEBOL

import StatsBase: entropy, WeightVec, sample
using Distributions: VonMises, Normal, cdf
using PyPlot: imshow, plot, xlabel, ylabel, contour, figure, pause, hold
using Cubature
export figure

export SearchDomain, initial_belief, update_belief!, transition!
export Vehicle, VehicleSet
export O, observe, true_bearing
export fisher, EID
export plot_b, plot_eid, plot_mi
export plot_eid2, plot_eid3, meshgrid, plot_sim
export h_ij, hk2, update_phik!, phi_ij, fk
export ErgodicManager
export get_action, RandPolicy, ConstantPolicy, SMC, SMC2, FisherGreedy
export FisherGreedy2
export Simulation
export mutual_information

typealias Obs Int64
typealias Observation Obs
typealias ObsSet Vector{Obs}
typealias Action NTuple{2, Float64}
typealias ActionSet Vector{Action}
typealias Belief Matrix{Float64}

include("vehicleset.jl")
include("searchdomain.jl")
include("observations.jl")
include("ergodicity.jl")
include("eid.jl")
include("policy.jl")
include("simulation.jl")
include("infotheoretic.jl")
include("plotting.jl")

# TODO: add checks for staying within
"""
`transition(m, x, a)`

Returns vehicle state after taking action `a`. Not currently working.
"""
function transition!(m::SearchDomain, X::VehicleSet, A::ActionSet)
	for i = 1:length(X)
		X[i].x = min(max(X[i].x + A[i][1], 0), m.num_cells)
		X[i].y = min(max(X[i].y + A[i][2], 0), m.num_cells)
	end
end

# Finds the next closest cell you end up at
function nextcell(m::SearchDomain, v::Vehicle, a::Action)
	# Determine grid cell you belong to
	grid_x = round(Int, v.x + a[1], RoundDown)
	grid_y = round(Int, v.y + a[2], RoundDown)

	# Keep grid cell within bounds
	grid_x = min(max(grid_x, 0), m.num_cells-1)
	grid_y = min(max(grid_y, 0), m.num_cells-1)
	return grid_x, grid_y
end


end # module
