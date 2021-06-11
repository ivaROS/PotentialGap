### Swept Gap Prioritization

| Parameter |        Value      | Meaning |
| --------- | ----------------- | ------- |
| d<sub>max</sub> | X | Maximum value of egocircle (or laser scan) range values. Mostly determined by the sensor. |
| r<sub>ins</sub> | X | Inscribed radius of the robot. Determined by geometry of the robot. |
| &tau;<sub>a</sub> | 3&pi;/4 | Threshold on angle between the gap chord and the shortest side (define by the closest gap edge point to robot). Determines what gaps are classified as radial gaps. |
| c<sub>a</sub> | &pi;/2 | Threshold on acceptable angular difference between most distant gap edge points for merging two gaps. |
| c<sub>d</sub> | XX | Threshold on acceptable range difference between most distance gap edge points for merging two gaps. Recommended to lie in the range  [0.5,2] r<sub>ins</sub>. |
