######################################################################
# plotting.jl
# Handles all the calls to PyPlot to generate plots
######################################################################

function plot_world(m::SearchDomain, X::VehicleSet)
	plot_world(m.b, X, m.theta)
end

function plot_world(b::Belief, X::VehicleSet, theta::NTuple{2,Int})
	#figure()
	mark_size = 12
	plot(theta[1], theta[2], "r^", markersize=mark_size)
	hold(true)
	for xi in X
		plot(xi.x, xi.y, "b*", markersize=mark_size)
	end
	imshow(b', interpolation="none", cmap="Greys", origin="lower")
	xlabel("x")
	ylabel("y")
end

function plot_sim(m::SearchDomain, s::Simulation)
	for t = 0:s.T
		hold(false)
		pause(1)
		plot_world(s.belief_list[t+1], s.state_list[t+1], m.theta)
	end
end

function plot_eid(m, b, x, theta)
	mark_size = 12
	eid = EID(m,b)
	plot(x[1], x[2], "b*", markersize=mark_size)
	plot(theta[1], theta[2], "r^", markersize=mark_size)
	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
	plot_contour(m, eid)
	xlabel("x")
	ylabel("y")
	#imshow(b_plot, cmap="Greys")
end

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

"""
`contour(m::SearchDomain, d)`

Plots contours of some distribution `d` (a matrix).
"""
function plot_contour(m::SearchDomain, d)
	X,Y = meshgrid(0:m.num_cells-1, 0:m.num_cells-1)
	contour(X, Y, d')
end
