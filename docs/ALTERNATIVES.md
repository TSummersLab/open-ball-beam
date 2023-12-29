# Alternative Ball & Beam systems

## Quanser Ball and Beam

- Commercial product available from <https://www.quanser.com/products/ball-and-beam/> .
- Price not publicly available, but indications are that the cost runs in the several thousand dollar range. <https://ftp.esat.kuleuven.be/stadius/barrero/cacsd/www%20and%20mails/EDU_Pricing.pdf>
- Proprietary control software, tethered to a company for support and under-the-hood knowledge.

## BOBShield

Recently BOBShield was proposed:

- <https://ieeexplore.ieee.org/abstract/document/9454013>
- <https://github.com/gergelytakacs/AutomationShield/wiki/BoBShield>

Similarities between BOBShield and OBB:

- Arduino-based
- Open-source
- Time-of-flight sensor

Benefits of BOBShield over OBB:

- Smaller
- Lower cost
- Lower part count
- Ball cannot fall off

Drawbacks of BOBShield under OBB:

- Tube/beam length is very small.
  - Limited operational range.
  - Harder to visualize large excursions.
- Servo is directly attached to the beam.
  - Beam angle to servo angle ratio is 1:1, which results in a lack of precision.
  - Beam angle only needs to be +/- 5 degrees in typical operation, while servo can easily do +/- 90 degrees.
  - Servo cannot rotate through its full range without collision.
    - By contrast, OBB the servo can rotate 360 degrees without issue.
    - Good during troubleshooting in case servo is accidentally commanded/activated to large angles.
- Servo is driven by Arduino power rather than an external power source
  - BOBShield can get away with this because they use a micro servo, but not ideal for reliable power delivery
- Ball is captured inside a transparent tube.
  - Can lead to unusual friction behavior (harder to control).
  - ToF sensor not able to function properly due to occlusions (harder to sense).
- No IMU sensor.
  - Cannot automatically calibrate servo angle against Earth's gravity.

## PyMoskito

<https://pymoskito.readthedocs.io/>

- Purely a software package.
- Has a Ball & Beam model <https://pymoskito.readthedocs.io/en/stable/examples/ballbeam.html>
- Also has several other dynamical systems.
