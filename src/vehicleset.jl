######################################################################
# vehicleset.jl
# v
######################################################################
type Vehicle
	x::Float64
	y::Float64
	max_v::Float64

	Vehicle(x::Float64, y::Float64, max_v::Float64) = new(x,y,max_v)
	Vehicle(x::Float64, y::Float64) = new(x,y,4.0)
end

typealias VehicleSet Vector{Vehicle}

type VehicleSet2
	vehicles::Array{Vehicle}
end
