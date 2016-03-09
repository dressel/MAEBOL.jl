######################################################################
# ergodicity.jl
# Handles the stuff needed for ergodicity
######################################################################

# TODO: need initial robot positions to initialize ck
# Gamma		constant weights
# h			constant normalizing weights
# phik		Fourier coefficients for current EID
# ck
type ErgodicManager
	K::Int
	L::Int
	Gamma::Matrix{Float64}
	h::Matrix{Float64}
	phi::Matrix{Float64}
	phik::Matrix{Float64}
	ck::Matrix{Float64}
	F

	function ErgodicManager(m::SearchDomain, K::Int)
		L = m.num_cells
		F = m.F

		# create data structures
		Gamma = zeros(K+1, K+1)
		h     = zeros(K+1, K+1)
		phik  = zeros(K+1, K+1)
		ck    = zeros(K+1, K+1)
		phi	  = EID(m)

		pil2 = pi * pi / (L * L)
		for K1 = 0:K
			for K2 = 0:K
				Gamma[K1+1, K2+1] = 1 / ( (1 + pil2*(K1*K1 + K2*K2))^1.5 )
				h[K1+1, K2+1] = h_ij(K1, K2, L)
			end
		end
		return new(K, L, Gamma, h, phi, phik, ck, F)
	end
end

"""
`fk(em, k1, k2, x1, x2)`

Arguments:

* `em` = ergodic manager
* `k1, k2` = coefficients (0-indexed)
* `x1, x2` = vehicle location

"""
function fk(em::ErgodicManager, K1::Int, K2::Int, x1, x2)
	return cos(K1*pi*x1/em.L) * cos(K2*pi*x2/em.L) / em.h[K1+1, K2+1]
end


######################################################################
# Trajectory Coefficients c_k
######################################################################
# t = 0 is the start point ... this method would fail...
# t = 1 is when we can start using this function
function update_ck!(em::ErgodicManager, X::VehicleSet, t::Int)
	for K1 = 0:em.K
		for K2 = 0:em.K
			em.ck[K1+1,K2+1] = ck_ij(em, K1, K2, X, t)
		end
	end
end

# Computes update for c_{ij}
# TODO: handle mulitple vehicles
function ck_ij(em::ErgodicManager, K1::Int, K2::Int, X::VehicleSet, t::Int)
	N = length(X)
	new_sum = 0.0
	for i = 1:N
		new_sum += fk(em, K1, K2, X[i].x, X[i].y)
	end
	return (N*em.ck[K1+1,K2+1] + new_sum) / (N*t)
end


"""
`h_ij(k1::Int, k2::Int, L::Int)`

`L` is size of search domain.
"""
function h_ij(K1::Int, K2::Int, L::Int)
	# Limits
	xmin = (0.,0.)
	xmax = (L, L)

	# make integrand
	function h_int(x)
		cos(K1*pi*x[1]/L)^2 * cos(K2*pi*x[2]/L)^2
	end

	return sqrt(hcubature(h_int, xmin, xmax)[1])
end

# This actually works remarkably well
function hk2(m::SearchDomain, k::Int)
	L = m.num_cells
	hksum = 0.0
	for x1 = 0.5:(L-.5)
		for x2 = 0.5:(L-.5)
			hksum += cos(k*pi*x1/L)^2 * cos(k*pi*x2/L)^2
		end
	end
	return sqrt(hksum)
end


######################################################################
# wtf
######################################################################
function update_phik!(em::ErgodicManager)
	K = em.K
	for K1 = 0:K
		for K2 = 0:K
			em.phik[K1+1, K2+1] = phi_ij(em, K1, K2)
		end
	end
end

"""
`phik(m::SearchDomain, phi::Matrix{Float64}, k::Int)`

Currently performs a rough numerical integration.
"""
function phi_ij(em::ErgodicManager, K1::Int, K2::Int)
	L = em.L
	phiksum = 0.0
	k1 = K1 * pi / L
	k2 = K2 * pi / L
	for x1 = 1:L
		for x2 = 1:L
			phiksum += em.phi[x1, x2] * cos(k1*(x1-.5)) * cos(k2*(x2-.5))
		end
	end
	return phiksum / em.h[K1+1, K2+1]
end

# Returns bx, by, the two components of Bj
function Bj(em::ErgodicManager, x1, x2, N, t)
	bx = 0.
	by = 0.
	for K1 = 0:em.K
		for K2 = 0:em.K
			k1 = K1 * pi / em.L
			k2 = K2 * pi / em.L

			Sk = N*t*(em.ck[K1+1,K2+1] - em.phik[K1+1,K2+1])
			m = em.Gamma[K1+1,K2+1] * Sk

			bx += -k1 * m * sin(k1*x1) * cos(k2*x2) / em.h[K1+1, K2+1]
			by += -k2 * m * cos(k1*x1) * sin(k2*x2) / em.h[K1+1, K2+1]
		end
	end
	return bx,by
end

function uj(em::ErgodicManager, x1, x2, t)
	bx, by = Bj(em, x1, x2, t)
	bnorm = sqrt(bx*bx + by*by)
end

function update_phi!(em::ErgodicManager, b::Belief)
	n = em.L
	F = em.F
	for vx = 1:n
		for vy = 1:n
			ma = 0.0; mb = 0.0; mc = 0.0; md = 0.0
			for theta_x = 1:n
				for theta_y = 1:n
					p = b[theta_x, theta_y]
					ma += F[vx, vy, theta_x, theta_y,1,1] * p
					mb += F[vx, vy, theta_x, theta_y,1,2] * p
					mc += F[vx, vy, theta_x, theta_y,2,1] * p
					md += F[vx, vy, theta_x, theta_y,2,2] * p
				end
			end
			em.phi[vx, vy] = ma*md - mb*mc
		end
	end
end
