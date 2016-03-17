######################################################################
# plotting.jl
# Handles all the calls to PyPlot to generate plots
######################################################################

"""
`plot_b(m::SearchDomain, X::VehicleSet)`

Plots the belief, jammer, and vehicles.
"""
function plot_b(m::SearchDomain, X::VehicleSet)
	#hold(true)
	#plot_b(m.b, X, m.theta)
	plot_theta(m)
	plot_vehicles(X)
	imshow(m.b', interpolation="none", cmap="Greys", origin="lower")
	labels()
end

# This is needed for plot_sim
function plot_b(m::SearchDomain, b::Belief, X::VehicleSet)
	plot_theta(m)
	hold(true)
	plot_vehicles(X)
	imshow(b', interpolation="none", cmap="Greys", origin="lower")
	labels()
end

"""
`plot_eid(m::SearchDomain, X::VehicleSet)`

Plots the eid using the Fisher information matrix.
"""
function plot_eid(m::SearchDomain, X::VehicleSet)
	eid = EID(m,m.b)
	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
	plot_contour(m, eid)
	plot_theta(m)
	plot_vehicles(X)
	labels()
end

"""
`plot_mi(m::SearchDomain, X::VehicleSet)`

Plots the mutual information.
"""
function plot_mi(m::SearchDomain, X::VehicleSet)
	mut_info = mutual_information(m)
	plot_vehicles(X)
	imshow(mut_info', interpolation="none", cmap="Greys", origin="lower")
end

"""
`plot_sim(m::SearchDomain, s::Simulation)`

Steps through a simulation.
"""
function plot_sim(m::SearchDomain, s::Simulation)
	for t = 0:s.T
		hold(false)
		pause(1)
		plot_b(m, s.belief_list[t+1], s.state_list[t+1])
	end
end


######################################################################
# Helper functions
######################################################################
# Plots locations of the vehicles
function plot_vehicles(X::VehicleSet)
	mark_size = 12
	for xi in X
		plot(xi.x, xi.y, "b*", markersize=mark_size)
	end
end

# Plots jammer location
function plot_theta(m::SearchDomain)
	mark_size = 12
	plot(m.theta[1], m.theta[2], "r^", markersize=mark_size)
end

# Plots contours of some distribution `d` (a matrix).
function plot_contour(m::SearchDomain, d)
	X,Y = meshgrid(0:m.num_cells-1, 0:m.num_cells-1)
	contour(X, Y, d')
end

# Sets the appropriate plot labels
function labels()
	xlabel("x")
	ylabel("y")
end

######################################################################
# Old, deprecated, or unused
######################################################################
function plot_eid2(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	plot(x[1], x[2], "b*", markersize=mark_size)
	plot(theta[1], theta[2], "r^", markersize=mark_size)
	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
	xlabel("x")
	ylabel("y")
end

function plot_eid3(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	#plot(x[1], m.num_cells - 1 - x[2], "b*", markersize=mark_size)
	#plot(theta[1], m.num_cells - 1 - theta[2], "r^", markersize=mark_size)
	#eid_plot = (eid')[end:-1:1, :]

	#imshow(eid_plot, interpolation="none", cmap="Greys")
	plot_contour(m, eid)
	xlabel("x")
	ylabel("y")
end

"""
`meshgrid(x, y)`

Returns X and Y.
"""
function meshgrid(x, y)
	nx = length(x)
	ny = length(y)

	X = zeros(ny, nx)
	Y = zeros(ny, nx)
	for xi = 1:nx
		for xj = 1:ny
			X[xj, xi] = x[xi]
			Y[xj, xi] = y[xj]
		end
	end
	return X, Y
end
