######################################################################
# observations.jl
#
# Contains everything needed for the observation function.
######################################################################

"""
`O(m::SearchDomain, x::Vehicle, theta, o::Obs)`

Arguments:

 * `m` is a `SearchDomain`
 * `x` is a `Vehicle`
 * `theta` is a possible jammer location
 * `o` is an observation, 0 to 35

Returns probability of observing `o` from `(xp, theta)` in domain `m`.
"""
function O(m::SearchDomain, x::Vehicle, theta, o::Obs)

	# Calculate true bearing, and find distance to bin edges
	xp = (x.x, x.y)
	ang_deg = true_bearing(xp, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o)

	# now look at probability
	#p = cdf(m.d, deg2rad(rel_end)) - cdf(m.d, deg2rad(rel_start))
	p = cdf(m.d, rel_end) - cdf(m.d, rel_start)
	return p
end

function O(m::SearchDomain, xv::Float64, yv::Float64, theta, o::Obs)

	# Calculate true bearing, and find distance to bin edges
	xp = (xv, yv)
	ang_deg = true_bearing(xp, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o)

	# now look at probability
	#p = cdf(m.d, deg2rad(rel_end)) - cdf(m.d, deg2rad(rel_start))
	p = cdf(m.d, rel_end) - cdf(m.d, rel_start)
	return p
end


"""
`observe(m::SearchDomain, X::VehicleSet)`

Returns a set of possible observations for vehicles in set `X`.
	The output is a vector of observations, one per vehicle.
"""
function observe(m::SearchDomain, X::VehicleSet)
	n = length(X)
	Z = Array(Obs, 0)
	for xi in X
		obs_probs = Array(Float64, 0)
		for o in 0:35
			push!(obs_probs, O(m, xi, m.theta, o))
		end
		push!(Z, sample(0:35, WeightVec(obs_probs)))
	end
	return Z
end


# Find the relative offset
function rel_bin_edges(bearing_deg, o::Obs)

	# calculate start, end degrees of bin
	start_deg, end_deg = bin2deg(o)

	# compute relative distance to true bearing
	rel_start = fit_180(bearing_deg - start_deg)
	rel_end = fit_180(bearing_deg - end_deg)

	# Make sure start is further left on number line
	if rel_end < rel_start
		temp = rel_start
		rel_start = rel_end
		rel_end = temp
	end

	# Handle odd case where we stradle 180 degrees off
	if (rel_start < 0) && (rel_end > 100)
		rel_start = rel_end
		rel_end += 10
	end

	return rel_start, rel_end
end


# Fits an angle into -180 to 180
# Assume the angle is within -360 to 360
function fit_180(angle)
	if angle > 180
		angle = 360 - angle
	elseif angle < -180
		angle += 360
	end
	return angle
end


# Find the true angle between UAV and jammer.
#
# Parameters:
#  xp - location of vehicle
#  theta - location of jammer
#
# Returns true angle, measured from north, in degrees.
function true_bearing(xp, theta)

	xr = theta[1] - xp[1]
	yr = theta[2] - xp[2]
	return mod(rad2deg(atan2(xr,yr)), 360)
end

# What bin does degree fall into?
function findbin(degree)
	deg_bin = round(Int, degree / 10, RoundNearestTiesAway)
	if deg_bin == 36
		deg_bin = 0
	end
	return deg_bin
end


# returns (start_deg, end_deg) integer tuple
function bin2deg(bin_deg::Int64)
	if bin_deg == 0
		start_val = -5
		end_val = 5
	else
		start_val = 10bin_deg - 5
		end_val  = 10bin_deg + 5
	end
	return (start_val, end_val)
end
