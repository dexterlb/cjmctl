# cjmctl

A collection of utilities to help with motor/motion control.

Still work in progress, but the baseline functionality is usable.

### Roadmap
- [x] Velocity estimator: given raw position measurements, estimate current velocity
- [x] Velocity controller: dynamically adjust output torque/current/pwm to achieve set velocity
- [x] Position controller: dynamically adjust output velocity to reach given position (uses a "dumb" proportional control algorithm that works for most cases)
- [ ] Constant jerk trajectory planner: a position controller with proper trajectory planning


### Contributions

Development of cjmctl was supported, in part, through the OpenAgri project
(https://horizon-openagri.eu/), which has received funding from the European
Union's Horizon Europe research and innovation programme under Grant Agreement
no. 101134083.
