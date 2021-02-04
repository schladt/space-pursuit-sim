# space-pursuit-sim

```
usage: space-pursuit-sim.py [-h] [--defender-position DEFENDER_POSITION] [--defender-speed DEFENDER_SPEED] [--attacker-position ATTACKER_POSITION] [--attacker-speed ATTACKER_SPEED] [--goal GOAL] [--theta THETA] [--cannon-range CANNON_RANGE]

3d Space Pursuit Simulation

optional arguments:
  -h, --help            show this help message and exit
  --defender-position DEFENDER_POSITION
                        String (csv) representing the defender's starting position (default '0,0,0')
  --defender-speed DEFENDER_SPEED
                        Int of the defender's speed (default 1
  --attacker-position ATTACKER_POSITION
                        String (csv) representing the attacker's starting position (default '-5,-5,-5')
  --attacker-speed ATTACKER_SPEED
                        Int of the attacker's speed (default 2
  --goal GOAL           String (csv) representing the defender's goal (default '10,10,10'
  --theta THETA         Attacker's cannon blast angle in degrees (default 45)
  --cannon-range CANNON_RANGE
                        Range (as int) of the attacker's cannon (default equal to attacker speed)
```