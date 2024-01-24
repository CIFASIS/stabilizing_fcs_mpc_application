This repository contains Simulink models and Matlab scripts and functions to run the experiments presented in:

R. Comelli, S. Olaru, M. M. Seron and E. Kofman, **Application of a stabilizing model predictive controller to path following for a car-like agricultural robot**, Optimal Control Applications and Methods. Under revision.

[comment]: <> (When published, add information to cite this and the other papers, something like:)
[comment]: <> (```bibtex)
[comment]: <> (@article{comelli2023application,)
[comment]: <> ( author = {Comelli, Román and Olaru, Sorin and Seron, Maria M. and Kofman, Ernesto},)
[comment]: <> ( title = {Application of a stabilizing model predictive controller to path following for a car-like agricultural robot},)
[comment]: <> ( journal = {Optimal Control Application and Methods},)
[comment]: <> ( pages = {?},)
[comment]: <> ( keywords = {model predictive control, path following, stability, mobile robotics, precision agriculture},)
[comment]: <> ( year = {2023},)
[comment]: <> ( doi = {https://doi.org/???},)
[comment]: <> ( url = {https://onlinelibrary.wiley.com/doi/abs/???},)
[comment]: <> ( eprint = {https://onlinelibrary.wiley.com/doi/pdf/???})
[comment]: <> (})
[comment]: <> (```)

The paper introduces a theoretical extension to use different control and prediction horizons in a recently developed MPC scheme that instead of a control invariant set, utilizes a pair of inner and outer sets with certain properties to guarantee finite-time convergence and ultimate boundedness to a target set. A controller for a car-like robot in a path following application is then designed, based on this strategy, and simulations and a comparison with another MPC approach are presented.

The previously developed ideas were presented in:

- R. Comelli, S. Olaru and E. Kofman, **Inner-Outer Approximation of Robust Control Invariant Sets**, Automatica, vol. 159, p. 111350, 2024.

- R. Comelli, A. H. González, A. Ferramosca, S. Olaru, M. M. Seron and E. Kofman, **Simplified Design of Practically Stable MPC Schemes**, Systems & Control Letters, vol. 180, p. 105626, 2023.

Executing the script `run_experiments.m`, you can try the proposed controller. It will perform many simulations with different combinations of prediction and control horizon and it will save results in folders within directories `fcs_mpc` and `nmpc`. To obtain the figure with trajectories given in the paper, go to `figures` and execute `plot_trajectories.m`.

Take into account that this was run in Matlab R2021b with Ubuntu 20.04.
