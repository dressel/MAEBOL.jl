######################################################################
# observations.jl
#
# Contains everything needed for the observation function.
######################################################################

"""
`O(m::SearchDomain, xp, theta, o::Obs)`

Arguments:

 * `xp` is (xv, yv), a tuple of doubles
 * `theta` is (theta_x, theta_y), a tuple of ints
 * `o` is an observation, 0 to 35

Returns probability of observing `o` from `(xp, theta)` in domain `m`.
"""
function O(m::SearchDomain, xp, theta, o::Obs)

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(xp, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o)

	# now look at probability
	#p = cdf(m.d, deg2rad(rel_end)) - cdf(m.d, deg2rad(rel_start))
	p = cdf(m.d, rel_end) - cdf(m.d, rel_start)
	return p
end

"""
Samples an observation
"""
function observe(m::SearchDomain, xp, theta)
	obs_probs = Array(Float64, 0)
	for o in 0:35
		push!(obs_probs, O(m, xp, theta, o))
	end
	return sample(0:35, WeightVec(obs_probs))
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
