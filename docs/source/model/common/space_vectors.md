# Space Vectors

## Definition

The space vector transformation is implemented in {func}`motulator.common.utils.abc2complex` and its inverse in {func}`motulator.common.utils.complex2abc`. We use peak-valued complex space vectors, marked with boldface in equations {cite}`Hin2024`. As an example, the stator current space vector is

```{math}
    :label: space_vector

    \iss = \frac{2}{3}\left(\iA + \iB\e^{\jj 2\pi/3} + \iC\e^{\jj 4\pi/3}\right) 
```

where $\iA$, $\iB$, and $\iC$ are the time-varying phase currents. The subscript s denotes stator quantities and the superscript s denotes stationary coordinates. The components are $\iss = i_\upalpha + \jj i_\upbeta$, represented in code as `i_s_ab`.

The space vector excludes the zero-sequence component

```{math}
    :label: zero_sequence

    i_0 = \frac{1}{3}\left(\iA + \iB + \iC\right)
```

Zero-sequence voltage exists at converter outputs, but zero-sequence current cannot flow ($i_0 = 0$) in delta-connected systems or when the star point is isolated. Therefore, zero-sequence voltage produces no power or torque.

(coordinate-transformation)=

## Coordinate Transformation

Models and controls often use synchronous coordinates aligned with the rotor or control system variables. Consider a general coordinate system at angle $\thetac$ relative to stationary coordinates, rotating at $\omegac$:

```{math}
    :label: gen_coordinates

    \frac{\D\thetac}{\D t} = \omegac 
```

The transformation to these coordinates is

```{math}
    :label: coordinate_transformation

    \is = \e^{-\jj\thetac}\iss
```

In rotor coordinates ($\thetac = \thetam$), components are traditionally denoted as $\is = \id + \jj \iq$, coded as `i_s_dq`. Controller coordinates use simply `i_s`. Similar notation applies to other space vectors.