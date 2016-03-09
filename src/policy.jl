######################################################################
# policy.jl
######################################################################
abstract Policy

type RandPolicy <: Policy
end

# Assumes max speed is 4 m/s, dt = 0.5 => 2 m/s
function get_action(m::SearchDomain, X::VehicleSet, p::RandPolicy, t)
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

# Spectral Multiscale Coverage
type SMC <:Policy
	em::ErgodicManager

	function SMC(m::SearchDomain, K)
		em = ErgodicManager(m, K)
		return new(em)
	end
end

function get_action(m::SearchDomain, X::VehicleSet, p::SMC, t)
	n = length(X)
	A = Array(NTuple{2,Float64}, n)
	
	# update EID, coefficients of phik, and ck
	update_phi!(p.em, m.b)
	update_phik!(p.em)
	update_ck!(p.em, X, t)

	# compute Bj, uj
	for i = 1:n
		bx, by = Bj(p.em, X[i].x, X[i].y, n, t)
		den = sqrt(bx*bx + by*by)
		ux = 2bx / den
		uy = 2by / den
		A[i] = (ux,uy)
	end
	return A
end
