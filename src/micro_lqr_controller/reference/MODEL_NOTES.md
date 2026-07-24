# Course-to-robot model mapping

The supplied MATLAB lesson uses a four-state linear model and `lqr(A,B,Q,R)`.
Its matrix structure implies the state order:

```text
[pitch, pitch_rate, position, velocity]
```

This replacement keeps that order throughout the model, gain matrix, debug output and runtime state vector.

The continuous model is written as:

```text
x_dot = A*x + B*u_total
```

where `u_total` is total axle torque. Wheel-ground force is `F=u_total/r`.

For body mass `m`, equivalent translating mass `M`, COM height `h`, body inertia about COM `I`, wheel radius `r`, and

```text
Delta = (M+m)*(I+m*h^2) - m^2*h^2
```

the default matrices are:

```text
A = [0,                         1, 0, 0;
     (M+m)*m*g*h/Delta,         0, 0, 0;
     0,                         0, 0, 1;
     -m^2*g*h^2/Delta,          0, 0, 0]

B = [0;
     -m*h/(Delta*r);
     0;
     (I+m*h^2)/(Delta*r)]
```

The original uploaded MATLAB file is preserved beside this note. Its fourth `B` entry can be selected with `model.use_course_legacy_b4: true` for numerical comparison. The hardware default uses the coefficient obtained by solving the coupled linear equations.

The ROS node performs exact zero-order-hold discretization and solves the discrete Riccati equation because the controller runs digitally at 333.333 Hz.
