## SIMA / PAMI : Eurobot 2026 "Winter Is Coming"

---

## 1. The Competition
Eurobot is a yearly robotics contest held in France where student and hobbyist teams build autonomous robots that compete head-to-head on a shared table. Each match lasts just 100 seconds, and every movement the robot makes has to happen on its own — no remote control once the game starts. The 2026 theme is "Winter Is Coming," and the story goes like this: you're a squirrel racing against another squirrel to grab as many hazelnut crates as possible before winter arrives. It sounds silly, but the engineering behind it is serious stuff.

A team typically brings two things to the table: a main robot that handles the complex strategy, and one or more smaller robots called PAMIs (or SIMAs in English). The main robot does the heavy lifting — navigating, grabbing, sorting, placing. The PAMI has a simpler life. Its job is usually to go from point A to point B, maybe push something over, and call it a day. Simple, reliable, done.

---

## 2. What Is a PAMI, Exactly?
PAMI stands for "Petit Actionneur Mobile Indépendant" — basically, a small independent mobile actuator. The English equivalent is SIMA. The rules around it are straightforward but strict. Once you launch it, it's on its own. You can trigger it with a pull-cord at the start of the match or have your main robot nudge a button mid-game, but after that moment, no radio, no wires, no helping hands. It has to figure out the rest by itself.

It also has to pass its own technical inspection, separate from the main robot. Same safety rules apply: emergency stop capability, can't ram the opponent, proper battery handling, all that good stuff. And it needs to work for either team color — you don't get to reprogram it between matches depending on which side you're assigned.

Because of all this, a PAMI's code doesn't need to be fancy. It doesn't need pathfinding algorithms or computer vision or complex decision trees. What it absolutely does need is rock-solid trajectory execution — the ability to take a sequence of moves and turn them into precise wheel motions, repeatably, without drifting off course. That's the core problem PAMI1 solves.

---

## 3. How PAMI1 Works
PAMI1 is a two-wheeled differential-drive robot, meaning it moves by spinning its left and right wheels at different speeds or directions, similar to how a wheelchair turns. The wheels are driven by stepper motors, which are a specific type of motor that moves in discrete steps rather than spinning smoothly. This is actually really useful for a PAMI because it means you can count exactly how many steps you've taken and know precisely how far you've traveled — no need for wheel encoders or odometry corrections.

The brain of the operation is an STM32 microcontroller, which is a popular choice in embedded systems. It handles all the timing-critical stuff through hardware timers: generating the step pulses for the motors, measuring the return signal from the ultrasonic sensor, and running the interrupt routines that keep everything synchronized.

## 3.1. Motion and Acceleration
When you tell a stepper motor to move, you can't just instantly blast it at full speed — the motor will skip steps and lose position. Instead, you accelerate gradually, cruise at your target speed, then decelerate to a stop. The code handles this with a trapezoidal speed profile. It calculates how many steps to spend accelerating, holds the target speed in the middle, then mirrors the acceleration phase for deceleration. All of this runs inside a timer interrupt, which fires at a high frequency and toggles the step pins at exactly the right moments.

## 3.2. From Distances to Steps
The robot doesn't think in meters or degrees — it thinks in steps. So there's a simple conversion layer: given the wheel radius, you can calculate how many steps it takes to travel one meter (wheel circumference times steps per revolution). For turning, you use the wheel base — the distance between the two wheels — to figure out how far each wheel needs to travel to rotate the robot by a given angle. The wheels spin in opposite directions during a turn, and the math works out cleanly.

## 3.3. Obstacle Detection
There's a single ultrasonic sensor on the front that pings out a sound wave and measures how long it takes to bounce back. If something is closer than about 20 centimeters, the code sets a flag that tells the motor interrupt routine to freeze all motion until the path is clear again. It's not smart avoidance — the robot doesn't try to go around anything. It just stops and waits. For a PAMI, that's usually enough.

## 3.4. The Trajectory System
This is the part that got refactored recently, and it's probably the most interesting piece of the codebase. Originally, the sequence of moves — go forward 40 centimeters, turn right 75 degrees, go forward 70 centimeters, and so on — was hardcoded directly in main as a long block of move calls and wait loops. It worked, but it was messy and adding a new path meant duplicating a bunch of code.

The new system is data-driven. A single move is described by a type (forward, backward, turn clockwise, turn counter-clockwise) and a value (distance in meters or angle in degrees). A trajectory is just an array of these moves. There are four trajectory slots available, corresponding to the different target positions the PAMI might need to reach depending on the match setup. One function, robot_execute_trajectory, walks through whatever trajectory you hand it and drives the motors accordingly. Main now just picks an index and calls that function — clean and simple.

## 3.5. Where Things Stand Right Now
Trajectory 0 is fully defined with real measured values: forward 40 centimeters, turn 75 degrees clockwise, forward 70 centimeters, turn 75 degrees counter-clockwise, forward 33 centimeters. This was pulled from the original working code, so it should produce the same behavior that was already tested on the table.

Trajectories 1 through 3 are still empty placeholders. The actual distances and angles for those paths need to be measured on the real table and filled in. The selection logic in main is also temporary — right now it just picks trajectory 0 or 1 based on a physical switch, but in a real match this would depend on which side of the table you're assigned and what your strategy calls for.

After the trajectory finishes, the robot currently just runs a servo sweep in the infinite loop, which is a placeholder for whatever end-of-match action gets added later — maybe deploying a flag, triggering a mechanism, or just sitting still.

