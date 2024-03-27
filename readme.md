This repository contains Simulink models and Matlab scripts and functions to run the experiments presented in:

```bibtex
@article{comelli2024application,
    author = {Comelli, Román and Olaru, Sorin and Seron, María M. and Kofman, Ernesto},
    title = {Application of a stabilizing model predictive controller to path following for a car-like agricultural robot},
    journal = {Optimal Control Applications and Methods},
    volume = {},
    number = {},
    pages = {},
    year = {2024},
    doi = {10.1002/OCA.3126},
    url = {https://onlinelibrary.wiley.com/doi/abs/10.1002/oca.3126},
    eprint = {https://onlinelibrary.wiley.com/doi/pdf/10.1002/oca.3126}
}
```

If something from this repository or the article is used in an academic work, please cite it.

The paper introduces a theoretical extension to use different control and prediction horizons in a recently developed MPC scheme that instead of a control invariant set, utilizes a pair of inner and outer sets with certain properties to guarantee finite-time convergence and ultimate boundedness to a target set. A controller for a car-like robot in a path following application is then designed, based on this strategy, and simulations and a comparison with another NMPC approach are presented.

The previously developed ideas were presented in:

- R. Comelli, S. Olaru and E. Kofman, **Inner-Outer Approximation of Robust Control Invariant Sets**, Automatica, vol. 159, p. 111350, 2024.

- R. Comelli, A. H. González, A. Ferramosca, S. Olaru, M. M. Seron and E. Kofman, **Simplified Design of Practically Stable MPC Schemes**, Systems & Control Letters, vol. 180, p. 105626, 2023.

By executing the script `run_experiments.m`, you can try the proposed controller. It will perform many simulations with different combinations of prediction and control horizons and it will save results in folders within directories `fcs_mpc` and `nmpc`. To obtain the figure with trajectories given in the paper, go to `figures` and execute `plot_trajectories.m`.

Take into account that this was run in Matlab R2021b with Ubuntu 20.04.
