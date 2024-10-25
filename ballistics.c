/*

Setting Projectile Properties

Each weapon has a characteristic muzzle velocity: the speed at which the projectile is emitted from the weapon. This will be very fast for a laser-bolt and probably
considerably slower for a fireball. For each weapon the muzzle velocity used in the
game is unlikely to be the same as its real-world equivalent.

If we want the projectile's motion to be visible, we use muzzle velocities that are in the region
of 5 to 25 m/s. This causes two consequences we have to cope with:

First, the mass of the particle should be larger than in real life, especially if we are working with the full physics engine and we want the impacts
to look impressive. The effect that a projectile has when it impacts depends on both it's mass and it's velocity: if we drop the velocity, we should
increase the mass to compensate. The equation that links energy, mass and speed is:

e = m * (s ^ 2) * (1 / 2)

where e is the energy and s is the speed of the projectile (this equation doesn't work with vectors so we can't use velocity). If we want to keep the same
energy, we can work out the change in mass for a known change in speed:

dm = (ds) ^ 2

Second, we have to decrease the gravity on projectiles. Most projectiles shouldn’t
slow too much in flight, so the damping parameter should be near 1. Shells and mortars may arch under gravity, but other types of projectiles should barely feel the effect.
If they were traveling at very high speed, then they wouldn’t have time to be pulled
down by gravity to a great extent, but since we’ve slowed them down, gravity will have longer to do its work. Likewise, if we are using a higher gravity coefficient in the game,
it will make the ballistic trajectory far too severe: well-aimed projectiles will hit the
ground only a few meters in front of the character. To avoid this we lower the gravity.
For a known change in speed we can work out a “realistic” gravity value using the
formula:

g_mod = g_correct / (ds)

*/

/* 

bullet:
mass = 2kg
velocity = 35i
acceleration = -1j
damping = 0.99

artillery:
mass = 200kg
velocity = 30i, 40j
acceleration = -20j
damping = 0.99

fireball:
mass = 1kg
velocity = 10i
acceleration = 0.6j
damping = 0.9

*/

#include "particle.h"
