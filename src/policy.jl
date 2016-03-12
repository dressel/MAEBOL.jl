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

type ConstantPolicy <: Policy
	n::Int
	A::Vector{Action}

	function ConstantPolicy(X::VehicleSet, A::Vector{Action})
		n = length(X)
		@assert n == length(A)
		for i = 1:n
			dx = A[i][1]
			dy = A[i][2]
			den = sqrt(dx*dx + dy*dy)
			A[i] = (2*dx/den, 2*dy/den)
		end
		return new(n, A)
	end
end

get_action(m::SearchDomain, X::VehicleSet, p::ConstantPolicy, t) = p.A

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
		ux = -2bx / den
		uy = -2by / den
		A[i] = (ux,uy)
	end
	return A
end

# Spectral Multiscale Coverage
type SMC2 <:Policy
	em::ErgodicManager

	function SMC2(m::SearchDomain, K)
		em = ErgodicManager(m, K)
		return new(em)
	end
end

function get_action(m::SearchDomain, X::VehicleSet, p::SMC2, t)
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
		ux = -2bx / den
		uy = -2by / den
		a = (ux,uy)
		A[i] = a

		# let next guy know where you end up
		grid_x, grid_y = nextcell(m, X[i], (0.,0.))
		p.em.phi[grid_x, grid_y] = 0.0
		grid_x, grid_y = nextcell(m, X[i], a)
		p.em.phi[grid_x, grid_y] = 0.0
		update_phik!(p.em)
	end
	return A
end


######################################################################
# FisherGreedy Policy
######################################################################

# A is a vector that get returned every time get_action is called
# possible_actions is a 
type FisherGreedy <:Policy
	em::ErgodicManager
	A::Vector{Action}
	possible_actions::Vector{Action}

	function FisherGreedy(m::SearchDomain, X::VehicleSet)
		n = length(X)
		em = ErgodicManager(m, 1)

		# Create A
		A = Array(Action, n)

		# create possible_actions
		possible_actions = Array(Action, 8)
		possible_actions[1] = (0., 1.)
		possible_actions[2] = (1., 1.)
		possible_actions[3] = (1., 0.)
		possible_actions[4] = (1., -1.)
		possible_actions[5] = (0., -1.)
		possible_actions[6] = (-1., -1.)
		possible_actions[7] = (-1., 0.)
		possible_actions[8] = (-1., 1.)

		for i = 1:8
			dx = possible_actions[i][1]
			dy = possible_actions[i][2]
			den = sqrt(dx*dx + dy*dy)
			possible_actions[i] = (2.0*dx/den, 2.0*dy/den)
		end

		return new(em, A, possible_actions)
	end
end

function get_action(m::SearchDomain, X::VehicleSet, p::FisherGreedy, t)
	n = length(X)
	
	# update EID, coefficients of phik, and ck
	@time update_phi!(p.em, m.b)

	# loop over all vehicles, consider all possible actions
	for i = 1:n
		best_phi = 0.0
		best_a = (0.0, 0.0)
		for a in p.possible_actions
			grid_x, grid_y = nextcell(m, X[i], a)
			if p.em.phi[grid_x, grid_y] > best_phi
				best_phi = p.em.phi[grid_x, grid_y]
				best_a = a
			end
		end
		p.A[i] = best_a
	end

	return p.A
end


type FisherGreedy2 <:Policy
	em::ErgodicManager
	A::Vector{Action}
	possible_actions::Vector{Action}

	function FisherGreedy2(m::SearchDomain, X::VehicleSet)
		n = length(X)
		em = ErgodicManager(m, 1)

		# Create A
		A = Array(Action, n)

		# create possible_actions
		possible_actions = Array(Action, 8)
		possible_actions[1] = (0., 1.)
		possible_actions[2] = (1., 1.)
		possible_actions[3] = (1., 0.)
		possible_actions[4] = (1., -1.)
		possible_actions[5] = (0., -1.)
		possible_actions[6] = (-1., -1.)
		possible_actions[7] = (-1., 0.)
		possible_actions[8] = (-1., 1.)

		return new(em, A, possible_actions)
	end
end

function get_action(m::SearchDomain, X::VehicleSet, p::FisherGreedy2, t)
	n = length(X)
	
	# update EID, coefficients of phik, and ck
	update_phi!(p.em, m.b)

	# loop over all vehicles, consider all possible actions
	for i = 1:n
		best_phi = 0.0
		best_a = (0.0, 0.0)
		for a in p.possible_actions
			grid_x, grid_y = nextcell(m, X[i], a)
			if p.em.phi[grid_x, grid_y] > best_phi
				best_phi = p.em.phi[grid_x, grid_y]
				best_a = a
			end
		end
		p.A[i] = best_a
		grid_x, grid_y = nextcell(m, X[i], best_a)
		p.em.phi[grid_x, grid_y] = 0.0
	end

	return p.A
end
