######################################################################
# searchdomain.jl
# Contains search domain type
######################################################################
type SearchDomain
	cell_size::Int64		# side of one grid cell, in meters
	num_cells::Int64		# number of cells per side
	domain_size::Int64
	num_vehicles::Int64
	noise_sigma::Float64  # std deviation of noise, degrees
	d::Normal
	F						# fisher information stuff
	b::Belief
	theta::NTuple{2, Int}

	function SearchDomain(num_cells::Int64, theta_x::Int, theta_y::Int)
		m = new()
		m.cell_size = 1
		m.num_cells = num_cells
		m.domain_size = num_cells
		m.num_vehicles = 1
		m.noise_sigma = 15.0
		m.d = Normal(0, m.noise_sigma)
		m.F = fisher(m)
		m.b = ones(num_cells, num_cells) / (num_cells * num_cells)
		m.theta = (theta_x, theta_y)
		return m
	end
	function SearchDomain(num_cells::Int64)
		return SearchDomain(num_cells, num_cells-1, num_cells-1)
	end

end

# noise_sigma is in degrees
function set_noise(m::SearchDomain, noise_sigma::Float64)
	m.noise_sigma = noise_sigma
	m.d = Normal(0, m.noise_sigma)
end

"""
`initial_belief(m::SearchDomain)`
"""
initial_belief(m::SearchDomain) = ones(m.num_cells, m.num_cells) / (m.num_cells * m.num_cells)


"""
`update_belief(m::SearchDomain, X::VehicleSet, Z::ObsSet)`

Modifies the `b` field of `m`.
"""
function update_belief!(m::SearchDomain, X::VehicleSet, Z::ObsSet)

	# Loop over all possible jammer positions
	bp_sum = 0.0
	for theta_x = 1:m.num_cells
		for theta_y = 1:m.num_cells
			temp_val = m.b[theta_x, theta_y]
			for (i,xi) in enumerate(X)
				temp_val *= O(m, xi, (theta_x-1,theta_y-1), Z[i])
			end
			m.b[theta_x, theta_y] = temp_val
			bp_sum += temp_val
		end
	end

	# normalize
	for theta_x = 1:m.num_cells
		for theta_y = 1:m.num_cells
			m.b[theta_x, theta_y] /= bp_sum
		end
	end
end

# allows you to initialize vehicle set
# Assumes there is an odd number of cells
function Vehicle(m::SearchDomain)
	mid_cell = (m.num_cells-1) / 2
	return Vehicle(mid_cell, mid_cell)
end
