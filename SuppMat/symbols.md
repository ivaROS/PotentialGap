### Gap Detection

| Symbol | Meaning |
| ------ | ------- |
| \mathcal{L} | Laser-scan like measurements |
| d(&middot;;&middot;) | Euclidean distance |
| &zeta;(j) | Coordinate of one measurement with index j |
| &theta;(j) | Scan angle associated to index j |
| \mathcal{G} | Gap set |
| G | Detected gap in the gap set |
| &alpha; | The angle to classify gap into swept or radial gap |
| SGP | Swept gap prioritization |
| CG | Closet gap |
| &#981; | Rotation angle of radial gaps in radial gap conversion |


### Potential Field Construction

| Symbol | Meaning |
| ------ | ------- |
| \mathcal{G}<sub>SGP</sub> | Gap set after swept gap prioritization |
| G&prime; | The gap after shrinking angle to ensure that the gap region is a polar triangle convex in Euclidian space. |
| x<sub>LG</sub><sup>*</sup> | Local goal point determined from the chosen gap |
| G<sup>*</sup> | The chosen gap |
| &Phi;(x) | Attractive potential. It will attract robot to the gap curve then through to the local goal. |
| &Theta;(x) | Purely rotational vector field |
| \mathbb{J} | Rotation matrix with so(2) skew-symmetric form |
| d<sup>&theta;</sup>(&middot;;&middot;) | Angular distance |
| &delta; | Parameter or symbol ? |
| p<sup>l/r</sup> |  |
| e<sub>⊥</sub> | Circulation term on the gap line. d<sup>&theta;</sup> is vanished. Circulation is purely perpendicular and inward pointing. |
| f<sup>&#981;</sup> | Other circulation term |
| e<sup>&rho;</sup> | Radially directed outward vector |

### Trajectory synthesis and scoring

| Symbol | Meaning |
| ------ | ------- |
| \mathcal{X} | A set of trajectories that integrated based on potential field. |
| \mathcal{T} | A single trajectory discretized into poses |
| p | Discretized poses on the trajectory |
| p<sub>*</sub> | Local navigation goal |
| p<sup>end</sup> | The last pose on the trajectory |
| J | Trajectory score |

### Projection Operator

| &xi;<sup>u</sup> | Feasible movement to control non-holonomic robot |
| &nu; | Linear velocity of non-holonomic robot |
| &omega; | Angular velocity of non-holonomic robot |
| \mathcal{C} | The set of collision curves to the left and to the right of the gap in polar space. |
| &psi; | Obstacle potential function |

