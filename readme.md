# Potential Gap: Using Reactive Policies to Guarantee Safe Navigation
This paper considers the integration of gap-based local navigation methods with artificial potential field (APF) methods to derive a local planning module for hierarchicalnavigation systems that has provable collision-free properties.Given that APF theory applies to idealized robot models, theprovable properties are lost when applied to more realistic models. We describe a set of algorithm modifications thatcorrect for these errors and enhance robustness to non-ideal models. Central to the construction of the local planner isthe use of sensory-derived local free-space models that detect gaps and use them for the synthesis of the APF. Modifications are given for a nonholonomic robot model. Integration of the local planner, called potential gap, into a hierarchical navigation system provides the local goals and trajectories needed for collision-free navigation through unknown environments.Monte Carlo experiments in benchmark worlds confirm the asserted safety and robustness properties by testing under various robot models.

[[**Demo Video**]](), [[**Arxiv Preprint**]](https://arxiv.org/abs/2103.11491)

# Supplementary materials
[Algorithm parameters](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/parameters.md), [Symbols](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/symbols.md), [Abbreviations](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/abbreviations.md)

# Dependencies and Installation

- ROS (Kinetic Ubuntu 16.04) [Installation Link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

See NavBench [https://github.com/ivalab/NavBench](https://github.com/ivalab/NavBench) for rosinstall instructions and launching experiments. 

# BibTex Citation
```
@misc{xu2021potential,
      title={Potential Gap: Using Reactive Policies to Guarantee Safe Navigation}, 
      author={Ruoyang Xu and Shiyu Feng and Patricio A. Vela},
      year={2021},
      eprint={2103.11491},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

# License
The source code is released under [MIT](https://opensource.org/licenses/MIT) license. 

# Code Release Todos
1. `turtlebot_trajectory_{generator, functions, controller, testing}`
2. `move_base_virtual`
3. `navigation_test` Update to STDR
4. Verify the removal of other dependencies
