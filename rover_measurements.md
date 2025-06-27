URDF UNITS: meters, radians, kilograms
URDF ORDER: Always x y z, so width, depth(length), and height

ROBOT ORIENTATION: Facing the RE-RASSOR, positive x is forward and
positive y is to the right and positive z is up

WHEEL LENGTH: 9.8cm -> 0.098m
WHEEL RADIUS: 11.5cm -> 0.115m

WHEEL TUBE LENGTH: 10cm -> 0.1
WHEEL TUBE RADIUS: 6.5cm -> 0.065

CHASSIS WIDTH: 37cm -> 0.37m
CHASSIS LENGTH: 19cm -> 0.19m
CHASSIS HEIGHT: 11cm -> 0.11m

base_link origin is the center of the robot on the xy axis and the bottommost part of the chassis on the z axis
OFFSET FROM:
middle of chassis to base link
    x: 0
    y: 0
    z: half of chassis height
fr tube to chassis
    x: half of chassis length - radius round down
    y: -i have no idea
    z: tude radius
fl tube to chassis
    x: half of chassis length - radius round down
    y: i have no idea
    z: tube radius
br tube to chassis
    x: -(half of chassis length - radius round down)
    y: i have no idea
    z: tube radius
bl tube to chassis
    x: -(half of chassis length - radius round down)
    y: -i have no idea
    z: tube radius
any wheel to any tube:
    x: 0
    y: half of tube length (some will be negative)
    z: 0