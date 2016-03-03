######################################################################
# ergodicity.jl
# Handles the stuff needed for ergodicity
######################################################################

# TODO: need initial robot positions to initialize ck
type ErgodicManager
	K::Int
	L::Int
	Gamma::Matrix{Float64}
	h::Matrix{Float64}
	phik::Matrix{Float64}
	ck::Matrix{Float64}

	function ErgodicManager(K::Int, L::Int)
		# create data structures
		Gamma = zeros(K+1, K+1)
		h = zeros(K+1, K+1)
		phik = zeros(K+1, K+1)
		ck = zeros(K+1, K+1)

		pil2 = pi * pi / (L * L)
		for k1 = 0:K
			for k2 = 0:K
				Gamma[k1+1, k2+1] = 1 / ( (1 + pil2*(k1*k1 + k2*k2))^1.5 )
				h[k1+1, k2+1] = h_ij(k1, k2, L)
			end
		end
		return new(K, L, Gamma, h, phik, ck)
	end
end

"""
`fk(em, k1, k2, x1, x2)`

Arguments:

* `em` = ergodic manager
* `k1, k2` = coefficients (0-indexed)
* `x1, x2` = vehicle location

"""
function fk(em::ErgodicManager, k1::Int, k2::Int, x1, x2)
	return cos(k1*pi*x1/em.L) * cos(k2*pi*x2/em.L) / em.h[k1+1, k2+1]
end

function rar()
end


"""
`h_ij(k1::Int, k2::Int, L::Int)`

`L` is size of search domain.
"""
function h_ij(k1::Int, k2::Int, L::Int)
	# Limits
	xmin = (0.,0.)
	xmax = (L, L)

	# make integrand
	function h_int(x)
		cos(k1*pi*x[1]/L)^2 * cos(k2*pi*x[2]/L)^2
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

"""
`phik(m::SearchDomain, phi::Matrix{Float64}, k::Int)`

Currently performs a rough numerical integration.
"""
function phik(m::SearchDomain, phi::Matrix{Float64}, k::Int)
	L = m.num_cells
	phiksum = 0.0
	for x1 = 1:L
		for x2 = 1:L
			phiksum += phi[x1, x2] * cos(k*pi*(x1-.5)/L)*cos(k*pi*(x2-.5)/L)
		end
	end
	return phiksum / hk(m, k)
end
