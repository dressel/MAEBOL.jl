# MAEBOL (Multi-agent, ergodic, bearing-only localization)
Here I consider searching for a single, stationary jammer using multiple vehicles with a bearing-only sensing modality.
I use the ergodic control framework presented in [1] as well as that presented in [2],[3].

This work is still in progress; this repo and documentation is really just for myself; nothing is guaranteed to work at all at any time.

## Installation and Use
To install, fire up Julia in the terminal and type
```
Pkg.clone("https://github.com/dressel/MAEBOL.jl.git")
```
To begin using, type the following while in Julia
```
using MAEBOL
```

## Main Types

### Search Domain
I use a `SearchDomain` type to characterize the search region in which the jammer might lie.

* `b`

To createa default `SearchDomain`, do the following
```
m = SearchDomain(40)
```

### Vehicle Set
A `VehicleSet` is simply a vector of `Vehicle` types.
The fields of vehicle are.

* `x`
* `y`

### Policy
You can define your own policies.

## Expected Information Density (EID)
The expected information density (EID) is a distribution over the search domain that characterizes the informational value of being at a specific point in the domain.

However, when we want to estimate a multivariable parameter, we have an expected information matrix (EIM).
We denote this matrix as Phi(x).

D-optimality is often used to convert the EIM into EID: EID(x) = det(Phi(x))

* `EID(m::SearchDomain, b::Belief)` generates EID matrix over domain.
* `EID(m::SearchDomain, theta_x, theta_y)` finds EID at a specific jammer location.

## Fourier Decomposition
In order to perform ergodic control, the EID needs to be broken down into Fourier coefficients.

`phik(m::SearchDomain, phi::Matrix{Float64}, k::Int)` finds the `k`th coefficient of the EDI represented with `phi`.


## Simulations
```
s = Simulation(m, X, p, 10)
```

## Sources

1. George Mathew, Igor Mezić, Metrics for ergodicity and design of ergodic dynamics for multi-agent systems, Physica D: Nonlinear Phenomena, Volume 240, Issues 4–5, 15 February 2011, Pages 432-442, ISSN 0167-2789, http://dx.doi.org/10.1016/j.physd.2010.10.010.
2. Y. Silverman, L. M. Miller, M. A. MacIver and T. D. Murphey, "Optimal planning for information acquisition," Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on, Tokyo, 2013, pp. 5974-5980.
3. L. M. Miller and T. D. Murphey, "Optimal planning for target localization and coverage using range sensing," Automation Science and Engineering (CASE), 2015 IEEE International Conference on, Gothenburg, 2015, pp. 501-508.

[![Build Status](https://travis-ci.org/dressel/MAEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/MAEBOL.jl)
