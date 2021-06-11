### Swept Gap Prioritization

| Parameter |        Value      | Meaning |
| --------- | :---------------: | ------- |
| d<sub>max</sub> | --| Maximum value of egocircle (or laser scan) range values. Mostly determined by the sensor. |
| r<sub>ins</sub> | -- | Inscribed radius of the robot. Determined by geometry of the robot. |
| &tau;<sub>a</sub> | 3&pi;/4 | Threshold on angle between the gap chord and the shortest side (define by the closest gap edge point to robot). Determines what gaps are classified as radial gaps. |
| c<sub>a</sub> | &pi;/2 | Threshold on acceptable angular difference between most distant gap edge points for merging two gaps. |
| c<sub>d</sub> | XX | Threshold on acceptable range difference between most distance gap edge points for merging two gaps. Recommended to lie in the range  [0.5,2] r<sub>ins</sub>. |
| &epsilon; = (&epsilon;<sup>1</sup>,&epsilon;<sup>2</sup>) | -- | Defines a vector with a base at the gap point and rotated relative to the gap curve tangent that would not be visible by the robot because it is on the other side of a radial gap. Rotating the radial gap and the point at the end of the vector about the gap edge point by the angle of this vector &phi; = atan(&epsilon;<sup>2</sup>/&epsilon<sup>1</sup>), makes the point line of sight visible. In doing so, the robot now has visibility on the other side of the transformed gap, which aids in the determination of a safe local goal point on the other side of the gap and the construction of a safe local trajectory. Usually scales with r<sub>ins</sub>|
