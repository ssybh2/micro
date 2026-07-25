# Offline validation report

Validation was performed against the model values in `config/lqr.yaml` with a 3-ms zero-order-hold discretization.

## Local unsaturated cascade gain

```text
K_equivalent = [-0.80, -0.04, -0.04, -0.04]
```

The local closed-loop poles are approximately:

```text
0.98108365 +/- 0.03409603j   |lambda|=0.98167595   zeta=0.4699   fd=1.8430 Hz
0.99871590 +/- 0.00257623j   |lambda|=0.99871922   zeta=0.4449   fd=0.1368 Hz
```

```text
spectral radius = 0.9987192223
radial margin   = 0.0012807777
```

## Simple nonlinear model check

A deterministic nominal-model check was also run with:

```text
initial pitch:       0 deg
initial position:    +0.40 m
initial velocity:    0 m/s
pitch request limit: +/-2 deg
pitch request slew:  5 deg/s
outer velocity LPF:  5 Hz
total torque limit:  +/-0.24 N*m
```

Nominal result:

```text
largest opposite-side position overshoot: about -0.083 m
maximum body pitch:                    about 2.10 deg
position at 5 s:                       about -0.045 m
position at 10 s:                      about -0.0014 m
position at 20 s:                      about -0.00003 m
```

This is not a real-hardware guarantee. The test excludes sensor delay, EtherCAT timing variation, motor dynamics, backlash, static friction, tire slip, floor slope, mass/inertia error, and sign errors. Its purpose is to catch algebra/sign mistakes and verify that the default nominal controller is locally stable and that the nonlinear pitch-request protection is active.
