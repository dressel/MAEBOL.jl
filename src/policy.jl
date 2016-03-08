######################################################################
# policy.jl
######################################################################
abstract Policy

type RandPolicy <: Policy
end
# Assumes max speed is 4 m/s, dt = 0.5 => 2 m/s
function get_action(m::SearchDomain, X::VehicleSet, p::RandPolicy)
	n = length(X)
	A = Array(NTuple{2,Float64}, n)
	for i = 1:n
		dx = 2.0*rand() - 1.0
		dy = 2.0*rand() - 1.0

		den = sqrt(dx*dx + dy*dy)
		dx = 2.0*dx/den
		dy = 2.0*dy/den
		A[i] = (dx, dy)
	end
	return A
end
