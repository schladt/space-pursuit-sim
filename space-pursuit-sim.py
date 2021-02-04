import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import argparse
import math

def main():
    
    # parse command line options
    parser = argparse.ArgumentParser(description='3d Space Pursuit Simulation')
    parser.add_argument('--defender-position', type=str, default="0,0,0", help="String (csv) representing the defender's starting position (default '0,0,0')")
    parser.add_argument('--defender-speed', type=int, default=1, help="Int of the defender's speed (default 1")
    parser.add_argument('--attacker-position', type=str, default="-5,-5,-5", help="String (csv) representing the attacker's starting position (default '-5,-5,-5')")
    parser.add_argument('--attacker-speed', type=int, default=2, help="Int of the attacker's speed (default 2")
    parser.add_argument('--goal', type=str, default="10,10,10", help="String (csv) representing the defender's goal (default '10,10,10'")
    parser.add_argument('--theta', type=int, default=45, help="Attacker's cannon blast angle in degrees (default 45)")
    parser.add_argument('--cannon-range', type=int, help="Range (as int) of the attacker's cannon (default equal to attacker speed)")

    args = parser.parse_args()

    # format arguments 
    def_pos = [int(x) for x in args.defender_position.split(',')]
    def_speed = args.defender_speed
    att_pos = [int(x) for x in args.attacker_position.split(',')]
    att_speed = args.attacker_speed
    goal = [int(x) for x in args.goal.split(',')]
    theta = args.theta
    cannon_range = args.cannon_range
    if cannon_range is None:
        cannon_range = att_speed

    # loop until either defender reaches goal or attacker black defender
    count = 0
    while True:
        # progess defender towards goal
        
        # let vector v be the vector from the defender to the goal
        v = [goal[0]-def_pos[0], goal[1]-def_pos[1], goal[2]-def_pos[2]]
        
        # v_m is magnnitude of v
        v_m = math.sqrt(v[0]**2 + v[1]**2 + v[2]**1)
        
        #v_n is the normalized vector multipled by the speed
        v_n = [x * def_speed / v_m for x in v]
        
        # v_p (v prime) is the next location the defender will be based on it's vector
        v_p = [v_n[0] + def_pos[0], v_n[1] + def_pos[1], v_n[2] + def_pos[2]]


        # now the defender will pursue the location of v prime
        # let vector u be the vector from the attacker to the defender's vector
         
        u = [v_p[0]-att_pos[0], v_p[1]-att_pos[1], v_p[2]-att_pos[2]]
        u_m = math.sqrt(u[0]**2 + u[1]**2 + u[2]**1)
        u_n = [x * att_speed / u_m for x in u]
        u_p = [u_n[0] + att_pos[0], u_n[1] + att_pos[1], u_n[2] + att_pos[2]]

        # plot starting position for attacker, defender, goal
        fig = plt.figure()
        
        ax = fig.add_subplot(111, projection='3d')
        plt.title("Positions at T={0}".format(count))

        # point plots
        ax.plot(def_pos[0],def_pos[1],def_pos[2],  marker='o', markersize=5)
        ax.plot(att_pos[0],att_pos[1],att_pos[2],  marker='o', markersize=5)
        ax.plot(goal[0],goal[1],goal[2],  marker='o', markersize=5)

        # quiver plots
        x, y, z = def_pos[0],def_pos[1],def_pos[2]
        u, v, w = v_n[0], v_n[1], v_n[2]

        x2, y2, z2 = att_pos[0],att_pos[1],att_pos[2]
        u2, v2, w2 = u_n[0], u_n[1], u_n[2]

        ax.quiver(x ,y, z, u, v, w)
        ax.quiver(x2 ,y2, z2, u2, v2, w2, colors='red')

        # label key points
        ax.text(def_pos[0],def_pos[1],def_pos[2], "Defender")
        ax.text(att_pos[0],att_pos[1],att_pos[2], "Attacker")
        ax.text(goal[0],goal[1],goal[2], "Goal")

        # label axis
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        plt.show()


        # determine if attacker can blast defender
        # let ad be the vector from the attacker to defender
        ad = [def_pos[0] - att_pos[0], def_pos[1] - att_pos[1], def_pos[2] - att_pos[2]]
        ad_mag = math.sqrt(ad[0]**2 + ad[1]**2 + ad[2]**2)
        if ad_mag < cannon_range:
            print("Attacker in range, checking blast angle")
            # we still have to check the angle between the normalized vector of the attacker (u_n) and the ad vector
            un_mag = math.sqrt(u_n[0]**2 + u_n[1]**2 + u_n[2]**2)
            angle = math.degrees(math.acos( (ad[0] * u_n[0] +  ad[1] * u_n[1] + ad[2] * u_n[2]) / (ad_mag * un_mag ) ))
            print("Blast angle is {0}".format(angle))
            if angle < theta:
                print("The attacker has blasted the defender at T={0}".format(count))
                break

        # determine if defender reached the goal
        if v_m < def_speed:
            print("Defender reached goal at T={0}".format(count + 1))
            break


        #update attacker and defender position
        def_pos = v_p
        att_pos = u_p
        del(v_p)
        del(u_p)

        count += 1

if __name__ == '__main__':
    main()