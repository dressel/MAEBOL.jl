# MAEBOL (Multi-agent, ergodic, bearing-only localization)
Here I consider searching for a single, stationary jammer using multiple vehicles with a bearing-only sensing modality.
I use the ergodic control framework presented in [1],[2].

## Search Domain
I use a `SearchDomain` type to characterize the search region in which the jammer might lie.

## Observation Function
```

```

## Expected Information Density (EID)
The expected information density (EID) is a distribution over the search domain that characterizes the informational value of being at a specific point in the domain.

However, when we want to estimate a multivariable parameter, we have an expected information matrix (EIM).
We denote this matrix as Phi(x).

D-optimality is often used to convert the EIM into EID: EID(x) = det(Phi(x))

* `EID(m::SearchDomain, b::Belief)` generates EID matrix over domain.
* `EID(m::SearchDomain, theta_x, theta_y)` finds EID at a specific jammer location.

## Sources

1. Y. Silverman, L. M. Miller, M. A. MacIver and T. D. Murphey, "Optimal planning for information acquisition," Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on, Tokyo, 2013, pp. 5974-5980.
2. L. M. Miller and T. D. Murphey, "Optimal planning for target localization and coverage using range sensing," Automation Science and Engineering (CASE), 2015 IEEE International Conference on, Gothenburg, 2015, pp. 501-508.

[![Build Status](https://travis-ci.org/dressel/MAEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/MAEBOL.jl)
