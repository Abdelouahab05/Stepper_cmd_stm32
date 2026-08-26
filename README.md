## SIMA / PAMI : Eurobot 2026 "Winter Is Coming"

---

## 1. The Competition
Eurobot is an annual French robotics competition where student and hobbyist teams build autonomous robots that compete on a shared arena. Matches last 100 seconds with no remote control allowed—robots must operate independently. The 2026 theme, "Winter Is Coming," casts competitors as squirrels racing to collect hazelnut crates before winter. Though playful in concept, the engineering is rigorous.

Teams deploy two robot types: a main robot that handles navigation, manipulation, and strategic placement, and one or more PAMIs (SIMAs), which perform simpler point to point tasks and basic actions like pushing objects. This division separates complex strategy from straightforward execution.

---

## 2. What Is a PAMI, Exactly?

PAMI (Petit Actionneur Mobile Indépendant) is a small, autonomous mobile actuator. Once activated—either by pull cord at match start or by the main robot pressing a button—it operates entirely independently with no radio control, wiring, or external intervention. Like the main robot, it must pass its own technical inspection and comply with safety regulations, including emergency stop capability and battery management. Importantly, it must function identically regardless of team color, requiring no reprogramming between matches. Unlike more complex systems, PAMI requires no pathfinding algorithms, computer vision, or elaborate decision making. Instead, it demands precise, repeatable trajectory execution—converting movement sequences into accurate wheel motions without drift. This core capability is what PAMI1 addresses.

---

## 3. How PAMI1 Works
PAMI1 is a differential drive robot that moves by spinning its wheels at different speeds, similar to a wheelchair turning. Stepper motors drive the wheels in discrete steps rather than continuous rotation, allowing precise distance measurement without wheel encoders or odometry corrections.

An STM32 microcontroller serves as the robot's control center, managing timing critical functions through hardware timers: generating motor step pulses, processing ultrasonic sensor signals, and executing interrupt routines to maintain system synchronization.

## 3.1. Motion and Acceleration
Stepper motors require gradual acceleration to avoid skipping steps and losing position. The solution is a trapezoidal speed profile: the motor accelerates to the target speed, maintains it, then decelerates symmetrically. A high frequency timer interrupt executes this sequence, toggling step pins at precise intervals.

## 3.2. From Distances to Steps
The robot measures movement in steps rather than meters or degrees. A simple conversion layer translates between units: using the wheel radius, you calculate how many steps equal one meter of travel. For turning, the wheel base determines how far each wheel must travel to rotate the robot by a specific angle. During turns, the wheels spin in opposite directions, making the calculations straightforward.

## 3.3. Obstacle Detection
A front mounted ultrasonic sensor detects obstacles by emitting sound waves and measuring their return time. When an object approaches within 20 centimeters, the sensor triggers a flag that halts all motor functions until the path clears. Rather than navigating around obstacles, the robot simply stops and waits a passive safety approach that suffices for a PAMI.

## 3.4. The Trajectory System
This refactored section is the codebase's most interesting component. Originally, the movement sequence—forward 40 cm, right 75°, forward 70 cm, etc. was hardcoded in main as repetitive move calls and wait loops. While functional, this approach was unwieldy and required significant code duplication to implement new paths.
The new system is data driven. A single move is described by a type (forward, backward, turn clockwise, turn counter clockwise) and a value (distance in meters or angle in degrees). A trajectory is just an array of these moves. There are four trajectory slots available, corresponding to the different target positions the PAMI might need to reach depending on the match setup. One function, robot_execute_trajectory, walks through whatever trajectory you hand it and drives the motors accordingly. Main now just picks an index and calls that function .

## 3.5. Where Things Stand Right Now
Trajectory 0 is fully defined with measured values from the original working code: forward 40 cm, turn 75° clockwise, forward 70 cm, turn 75° counter clockwise, forward 33 cm. This ensures tested behavior.

Trajectories 1–3 remain empty placeholders requiring field measurements. The selection logic currently uses a physical switch to choose between trajectories 0 and 1, but will ultimately depend on table assignment and match strategy.

After completing the trajectory, the robot executes an indefinite servo sweep as a placeholder for the final end-of-match action such as flag deployment, mechanism activation, or standby.

