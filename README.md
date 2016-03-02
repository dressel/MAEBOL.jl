# MAEBOL (Multi-agent, ergodic, bearing-only localization)

## Observation Function
```

```

## Expected Information Density (EID)
The expected information density (EID) is a distribution over the search domain that characterizes the value of being at a specific point in the domain.

However, when we want to estimate a multivariable parameter, we have an expected information matrix (EIM).
We denote this matrix as Phi(x).

D-optimality is often used to convert the EIM into EID: EID(x) = det(Phi(x))

* `EID(m::SearchDomain, b::Belief)` generates EID matrix over domain.
* `EID(m::SearchDomain, theta_x, theta_y)` finds EID at a specific jammer location.

## Sources

[![Build Status](https://travis-ci.org/dressel/MAEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/MAEBOL.jl)
