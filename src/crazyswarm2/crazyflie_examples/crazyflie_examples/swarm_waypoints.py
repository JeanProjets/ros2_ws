#!/usr/bin/env python
"""Exemple de vol en essaim rapide avec 4 drones parcourant une liste de waypoints."""

from crazyflie_py import Crazyswarm
import numpy as np

# Configuration
NUM_DRONES = 4
TAKEOFF_HEIGHT = 1.0
TAKEOFF_DURATION = 0.6     # ✔ accéléré
WAYPOINT_DURATION = 0.8    # ✔ accéléré
LAND_DURATION = 1.0        # ✔ accéléré

# Coordinates of waypoints
WAYPOINTS = [
    [1.0, 0.0, TAKEOFF_HEIGHT],
    [1.0, 1.0, TAKEOFF_HEIGHT],
    [0.0, 1.0, TAKEOFF_HEIGHT],
    [0.0, 0.0, TAKEOFF_HEIGHT],
]


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    if len(allcfs.crazyflies) < NUM_DRONES:
        print(f"Erreur: {len(allcfs.crazyflies)} drone(s) disponible(s), {NUM_DRONES} requis.")
        return

    drones = allcfs.crazyflies[:NUM_DRONES]

    print("Armement des drones...")
    for cf in drones:
        cf.arm(True)
    timeHelper.sleep(0.5)

    print("Décollage synchronisé...")
    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 0.2)   # ⏱ plus court

    print("Navigation rapide par waypoints...")
    for waypoint in WAYPOINTS:
        for cf in drones:
            target = np.array(cf.initialPosition) + np.array(waypoint)
            cf.goTo(target, yaw=0.0, duration=WAYPOINT_DURATION)
        timeHelper.sleep(WAYPOINT_DURATION + 0.2)  # ⏱ plus court

    print("Atterrissage synchronisé...")
    allcfs.land(targetHeight=0.05, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION + 0.5)

    print("Mission terminée !")


if __name__ == '__main__':
    main()
