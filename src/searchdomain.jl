type SearchDomain
	cell_size::Int64		# side of one grid cell, in meters
	num_cells::Int64		# number of cells per side
	domain_size::Int64
	num_vehicles::Int64
	noise_sigma::Float64  # std deviation of noise, degrees
	d::Normal
	F						# fisher information stuff

	function SearchDomain(num_cells::Int64)
		m = new()
		m.cell_size = 1
		m.num_cells = num_cells
		m.domain_size = num_cells
		m.num_vehicles = 1
		m.noise_sigma = 10.0
		m.d = Normal(0, m.noise_sigma)
		m.F = fisher(m)
		return m
	end
end

"""
`initial_belief(m::SearchDomain)`
"""
initial_belief(m::SearchDomain) = ones(m.num_cells, m.num_cells) / (m.num_cells * m.num_cells)


"""
`update_belief(m, x, b, a)`

Arguments:

* `m` is search domain
* `xp` is vehicle state
* `b` is belief matrix
* `o` is observation

Returns an updated belief matrix.
"""
function update_belief(m::SearchDomain, xp, b::Belief, o::Obs)
	bp = zeros(m.num_cells, m.num_cells)		# create bp

	# Loop over all possible jammer positions
	bp_sum = 0.0
	for theta_x = 1:m.num_cells
		for theta_y = 1:m.num_cells
			temp_val = O(m, xp, (theta_x-1,theta_y-1), o) * b[theta_x,theta_y]
			bp[theta_x, theta_y] = temp_val
			bp_sum += temp_val
		end
	end

	# normalize
	for theta_x = 1:m.num_cells
		for theta_y = 1:m.num_cells
			bp[theta_x, theta_y] /= bp_sum
		end
	end

	return bp
end
