######################################################################
# ergodicity.jl
# Handles the stuff needed for ergodicity
######################################################################

"""
`fisher(m::SearchDomain)`

Creates the Fisher information matrix. Vehicle positions are discretized to same level as possible jammer locations.
"""
function fisher(m::SearchDomain)
	# TODO: multiply by measurement noise
	# TODO: what do we do about rad2deg business? Multiply by constant?
	# TODO: what do we do about xr = yr = 0??
	n = m.num_cells
	F = zeros(n, n, n, n, 2, 2)

	for vx = 1:n
		for vy = 1:n
			for theta_x = 1:n
				for theta_y = 1:n
					# Create the 2x2 matrix
					xr = theta_x - vx
					yr = theta_y - vy
					if xr == 0 && yr == 0
						xr = 1e-6
						yr = 1e-6
					end
					den = (xr^2 + yr^2)^2
					F[vx,vy,theta_x,theta_y,1,1] = -2*yr*xr / den
					F[vx,vy,theta_x,theta_y,2,2] = 2*yr*xr / den
					F[vx,vy,theta_x,theta_y,1,2] = (xr^2 - yr^2) / den
					F[vx,vy,theta_x,theta_y,2,1] = (xr^2 - yr^2) / den
				end
			end
		end
	end
	return F
end


"""
`EID(m::SearchDomain, b::Belief)`

Arguments:

* `m` is the search domain
* `b` is the belief

Returns the expected information density.
"""
function EID(m::SearchDomain, b::Belief)
	n = m.num_cells
	F = m.F
	ei = zeros(n, n, 2, 2)
	eid = zeros(n, n)
	for vx = 1:n
		for vy = 1:n
			for theta_x = 1:n
				for theta_y = 1:n
					p = b[theta_x, theta_y]
					ei[vx, vy, 1, 1] += F[vx, vy, theta_x, theta_y,1,1] * p
					ei[vx, vy, 2, 1] += F[vx, vy, theta_x, theta_y,2,1] * p
					ei[vx, vy, 1, 2] += F[vx, vy, theta_x, theta_y,1,2] * p
					ei[vx, vy, 2, 2] += F[vx, vy, theta_x, theta_y,2,2] * p
				end
			end
			ma = ei[vx, vy, 1, 1]
			mb = ei[vx, vy, 2, 1]
			mc = ei[vx, vy, 1, 2]
			md = ei[vx, vy, 2, 2]
			eid[vx, vy] = ma*md - mb*mc
		end
	end
	return eid
end

"""
`EID(m::SearchDomain, theta_x::Int64, theta_y::Int64)`

Arguments:

* `m` is the search domain
* `(theta_x, theta_y)` is the jammer location (0-indexed)

Returns the expected information density.
"""
function EID(m::SearchDomain, theta_x::Int, theta_y::Int)
	n = m.num_cells
	F = m.F
	eid = zeros(n, n)
	for vx = 1:n
		for vy = 1:n
			ma = F[vx, vy, theta_x+1, theta_y+1, 1, 1]
			mb = F[vx, vy, theta_x+1, theta_y+1, 1, 2]
			mc = F[vx, vy, theta_x+1, theta_y+1, 2, 1]
			md = F[vx, vy, theta_x+1, theta_y+1, 2, 2]

			eid[vx, vy] = ma*md - mb*mc
		end
	end
	return eid
end


"""
`hk(m::SearchDomain, k::Int)`
"""
function hk(m::SearchDomain, k::Int)
	# Limits
	L = m.num_cells
	xmin = (0.,0.)
	xmax = (L, L)

	# make integrand
	function h_int(x)
		cos(k*pi*x[1]/L)^2 * cos(k*pi*x[2]/L)^2
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
`phik(m::SearchDomain, phi, k::Int)`

Currently performs a rough numerical integration.
"""
function phik(m::SearchDomain, phi, k::Int)
	L = m.num_cells
	phiksum = 0.0
	for x1 = 1:L
		for x2 = 1:L
			phiksum += phi(x1, x2) * cos(k*pi*(x1-.5)/L)*cos(k*pi*(x2-.5)/L)
		end
	end
	return phiksum / hk(m, k)
end
