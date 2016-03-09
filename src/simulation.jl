######################################################################
# simulation.jl
# Code for generating simulation traces
# 
# t = 0
#  s,o,b,a
# ...
# t = T
#  s,o,b
#
# T is the time length, so vectors will be T+1 length (0:T)
######################################################################

type Simulation
	T::Int
	state_list::Vector{VehicleSet}
	obs_list::Vector{ObsSet}
	belief_list::Vector{Belief}

	function Simulation(m0::SearchDomain, X::VehicleSet, p::Policy, T::Int)
		m = deepcopy(m0)

		# Create the structures:
		state_list = Array(VehicleSet, T+1)
		obs_list = Array(ObsSet, T+1)
		belief_list = Array(Belief, T+1)

		# t = 0
		Z = observe(m, X)
		update_belief!(m, X, Z)
		A = get_action(m, X, p, 1)

		state_list[1] = deepcopy(X)
		obs_list[1] = deepcopy(Z)
		belief_list[1] = copy(m.b)
		for t = 1:T
			transition!(m, X, A)
			Z = observe(m, X)
			update_belief!(m, X, Z)
			A = get_action(m, X, p, t)

			# Log it all...
			state_list[t+1] = deepcopy(X)
			obs_list[t+1] = deepcopy(Z)
			belief_list[t+1] = copy(m.b)
		end
		return new(T, state_list, obs_list, belief_list)
	end
end
