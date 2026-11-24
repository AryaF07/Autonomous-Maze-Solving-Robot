# Autonomous Maze-Solving Robot

Arduino (C++) firmware for a two-wheeled robot that navigates a maze on its own, built for RoboSoc.
It uses three fixed HC-SR04 ultrasonic sensors (left, front and right) and a rule-based left-hand wall follower with obstacle avoidance.

## How it works

Each loop, the robot reads all three sensors, then applies these rules in order:

1. **Turning:** if it is already turning away from a wall, it keeps rotating until the front is clear.
2. **Left-hand rule:** if the left wall it was following opens up, it turns left into the gap.
3. **Wall ahead:** it stops and turns toward whichever side has more room. At a dead end it backs up first.
4. **Corridor:** it drives forward, nudging away from any side wall that gets too close.

Sensor readings are made robust in two ways:

- **Median filter:** each sensor's reading is the median of 3 pings, so one bad echo can't trigger a turn.
- **Echo timeout:** `pulseIn` gives up after about 12 ms, and a missing echo counts as open space, so the loop never stalls.

## Wiring (Arduino Uno + L298N-style motor driver)

| Signal | Pin |
|---|---|
| ENA (left PWM) / ENB (right PWM) | 5 / 10 |
| IN1, IN2, IN3, IN4 | 9, 8, 7, 6 |
| Front sensor trig / echo | 13 / 12 |
| Left sensor trig / echo | 4 / 3 |
| Right sensor trig / echo | 11 / 2 |

## Tuning

These values depend on the motors, battery and maze, so set them on the real robot:

- **Distance thresholds:** `distance_collision`, `side_open` and `side_too_close`.
- **Timings:** `TURN_90_MS` and `FORWARD_INTO_GAP_MS`.

## Build

Open `FUME/FUME.ino` in the Arduino IDE, select **Arduino Uno**, and upload.
