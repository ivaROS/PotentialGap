# Potential Gap: A Gap-Informed Reactive Policy for Safe Hierarchical Navigation
This paper considers the integration of gap-based local navigation methods with artificial potential field (APF) methods to derive a local planning module for hierarchicalnavigation systems that has provable collision-free properties.Given that APF theory applies to idealized robot models, theprovable properties are lost when applied to more realistic models. We describe a set of algorithm modifications thatcorrect for these errors and enhance robustness to non-ideal models. Central to the construction of the local planner isthe use of sensory-derived local free-space models that detect gaps and use them for the synthesis of the APF. Modifications are given for a nonholonomic robot model. Integration of the local planner, called potential gap, into a hierarchical navigation system provides the local goals and trajectories needed for collision-free navigation through unknown environments.Monte Carlo experiments in benchmark worlds confirm the asserted safety and robustness properties by testing under various robot models.

[[**Demo Video**]](https://youtu.be/Im2aZdTpZHY), [[**Arxiv Preprint**]](https://arxiv.org/abs/2103.11491)

<img src="https://github.com/ivaROS/PotentialGap/blob/main/assets/coverImg.png" width = 55% height = 55%/>

## Supplementary materials

- [Algorithm parameters](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/parameters.md)
- [Manuscript symbols](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/symbols.md)
- [Manuscript abbreviations](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/abbreviations.md)
- [Links to main implementation code files](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/links_to_algorithm_sections.md)
- [Algorithm details of gap simplification, radial gap conversion and radial extension](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/gapMerging.pdf)
- [Comparison Between TEB and PG](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/compTable.pdf)
- [Proof Files for Passage](https://github.com/ivaROS/PotentialGap/blob/main/SuppMat/theorem.pdf)

# Dependencies and Installation

- ROS (Kinetic Ubuntu 16.04) [Installation Link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

See NavBench [https://github.com/ivalab/NavBench](https://github.com/ivalab/NavBench) for rosinstall instructions and launching experiments. 

<!-- - We are trying to release related dependencies as soon as possible. Please stay tuned -->

# BibTex Citation
```
@ARTICLE{9513583,
      author={Xu, Ruoyang and Feng, Shiyu and Vela, Patricio},
      journal={IEEE Robotics and Automation Letters},
      title={Potential Gap: A Gap-Informed Reactive Policy for Safe Hierarchical Navigation},
      year={2021},
      volume={},
      number={},
      pages={1-1},
      doi={10.1109/LRA.2021.3104623}
}
```

```
R. Xu, S. Feng and P. Vela, "Potential Gap: A Gap-Informed Reactive Policy for Safe Hierarchical Navigation," in IEEE Robotics and Automation Letters, doi: 10.1109/LRA.2021.3104623.
```

# License
The source code is released under [MIT](https://opensource.org/licenses/MIT) license. 

