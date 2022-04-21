# Open Ball & Beam (OBB)

*Control science for everyone.*

## Motivation
Existing educational control systems are often 
1. Prohibitively expensive (cost thousands of USD)
2. Use proprietary hardware and software
   - Development knowledge is not transferrable to other systems

OBB offers a viable alternative 
1. OBB is cheap enough for most individuals and labs
2. OBB uses open-source and widely available hardware and software



## Dependencies

Arduino
- [Arduino IDE](https://www.arduino.cc/en/software)
- [Servo library](https://www.arduino.cc/reference/en/libraries/servo/)
- [Polulu VL53L0X library](https://github.com/pololu/vl53l0x-arduino)
- [Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS)
- [BasicLinearAlgebra]()

Python
- [NumPy](https://numpy.org/)
- [SciPy](https://scipy.org/)
- [Matplotlib](https://matplotlib.org/)
- [PyQtGraph](https://www.pyqtgraph.org/)
- [PySerial](https://pythonhosted.org/pyserial/)
- [Keyboard](https://pypi.org/project/keyboard/)
- [ruamel.yaml](https://yaml.readthedocs.io/en/latest/)
- [OSQP](https://osqp.org/)

Install the dependency packages with the following Conda commands
- `conda install numpy scipy matplotlib`
- `conda install pyserial keyboard`
- `conda install ruamel.yaml`
- `conda install -c conda-forge pyqtgraph`
- `conda install -c conda-forge osqp`

TODO RL package? stable-baselines3? RRT* planning?

## Installation


### Arduino
1. Install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Install [Servo library](https://www.arduino.cc/reference/en/libraries/servo/) (may be installed already with the Arduino IDE)
    - Use the `Manage Libraries...` dialog in the Arduino IDE
3. [Polulu VL53L0X library](https://github.com/pololu/vl53l0x-arduino)
    - Use the `Manage Libraries...` dialog in the Arduino IDE
    - Be sure to use the Polulu library and NOT the Adafruit Library (regardless of whether the Polulu or the Adafruit physical hardware is used)
4. [Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS)
    - Use the `Manage Libraries...` dialog in the Arduino IDE
    - Search for `Adafruit LSM6DS`
    - Select `Install All`
5. [BasicLinearAlgebra]()
    - Use the `Manage Libraries...` dialog in the Arduino IDE
    - Search for `BasicLinearAlgebra`
7. When using `Tools -> Serial Monitor` make sure to use 115200 baud rate.

### Python
It is recommended to use Anaconda or virtualenv to create a separate environment to install in.

1. Activate the desired environment
2. Open a terminal / command prompt / Anaconda Prompt
3. Navigate to the root level directory of this package
4. Run the command `pip install .`


# Alternative Ball and Beam Systems

#### Quanser Ball and Beam
- Commercial product available from https://www.quanser.com/products/ball-and-beam/
- Price not publicly available, but indications are that the cost runs in the several thousand dollar range https://ftp.esat.kuleuven.be/stadius/barrero/cacsd/www%20and%20mails/EDU_Pricing.pdf
- Proprietary control software, tethered to a company for support and under-the-hood knowledge


#### BOBShield
Recently BOBShield was proposed
https://ieeexplore.ieee.org/abstract/document/9454013

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
- Tube/beam length is very small
  - Limited operational range
  - Harder to visualize large excursions
- Servo is directly attached to the beam
  - Beam angle to servo angle ratio is 1:1, which results in a lack of precision
  - Beam angle only needs to be +/- 5 degrees in typical operation, while servo can easily do +/- 90 degrees
  - Servo cannot rotate through its full range without collision
    - By contrast, OBB the servo can rotate 360 degrees without issue 
    - Good during troubleshooting in case servo is accidentally commanded/activated to large angles
- Servo is driven by Arduino power rather than an external power source
  - BOBShield can get away with this because they use a micro servo, but not ideal for reliable power delivery
- Ball is captured inside a transparent tube
  - Leads to weird friction behavior (harder to control)
  - ToF sensor not able to function properly due to occlusions (harder to sense)
- There is no IMU sensor
  - Cannot automatically calibrate servo angle against Earth's gravity
