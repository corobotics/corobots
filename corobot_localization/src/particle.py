import random
import math
from bresenhem import bresenhem
from kinect_loc import bres_condition


class Particle:
    slots = ("x_pos", "y_pos", "orientation", "probability")

    def __init__(self, x_exact, y_exact, orient_exact, mu, x_sigma, y_sigma, orientation_sigma):
        self.x_pos = x_exact + round(random.gauss(mu, x_sigma), 4) * random.randint(-1, 1)
        self.y_pos = y_exact + round(random.gauss(mu, y_sigma), 4) * random.randint(-1, 1)
        self.orientation = orient_exact + round(random.gauss(mu, orientation_sigma), 4)
        self.probability = 1 / 500

    def predict_update(self, trv_dist, new_orient, mu, dist_sigma, orient_sigma):
        est_trv_dist = trv_dist + round(random.gauss(mu, dist_sigma), 4) * random.randint(-1, 1)
        est_orient = new_orient + round(random.gauss(mu, orient_sigma), 4) * random.randint(-1, 1)
        self.x_pos = math.cos(self.orientation + new_orient) * est_trv_dist + self.x_pos
        self.y_pos = math.sin(self.orientation + new_orient) * est_trv_dist + self.y_pos
        self.orientation = round(math.fmod(self.orientation + est_orient), (2 * math.pi))

    def loc_check(self, map):
        resolution = map.info.resolution
        pixel_x = int(self.x_pos / resolution)
        pixel_y = int(self.y_pos / resolution)
        i = pixel_x + (map.info.height - pixel_y - 1) * map.info.width
        occ = map.data[i]
        if occ > 50:
            return False
        else:
            return True

    def probability_update(self, map, laser_scan):
        starting = laser_scan.angle_min
        ending = laser_scan.angle_max
        increment = laser_scan.angle_increment * 10

        # Calculated the expected Laser Scan results using bresenhem algorithm

        # Do calculations w.r.t the images pixels
        res = map.resolution
        ht = map.info.height

        # should convert robot pose into kinect pose (offset backward ~9 cm) first
        x_coord = self.x_pos / res
        y_coord = ht - (self.y_pos / res)
        starting_scan = self.orientation + starting
        ending_scan = self.orientation + ending

        current_scan = starting_scan

        scan = []

        # note theta is in original (right-handed) coords
        # to follow the scan order, we will increment in this coord system
        # but then negate the theta (or 2pi-theta) to do the image testing
        while current_scan <= ending_scan:
            scan_range = laser_scan.range_max / res
            target_x = x_coord + scan_range * math.cos(current_scan)
            target_y = y_coord + scan_range * math.sin(current_scan)

            if x_coord < 0:
                x_coord = 0
            elif x_coord > map.info.width:
                x_coord = map.info.width
            else:
                x_coord = int(x_coord)

            if y_coord < 0:
                y_coord = 0
            elif y_coord > map.info.height:
                y_coord = map.info.height
            else:
                y_coord = int(y_coord)

            if target_x < 0:
                target_x = 0
            elif target_x > map.info.width:
                target_x = map.info.width
            else:
                target_x = int(target_x)

            if target_y < 0:
                target_y = 0
            elif target_y > map.info.height:
                target_y = map.info.height
            else:
                target_y = int(target_y)

            distance = bresenhem(x_coord, y_coord, target_x, target_y, bres_condition)

            if distance[2] is True:
                actual_dist = math.sqrt((distance[0] - x_coord) ** 2 + (distance[1] - y_coord) ** 2) * res
                scan.append(actual_dist)
            else:
                scan.append(laser_scan.range_max)

            current_scan += increment

        # Compare expected and real data
            dist_diff = []
            scan_idx = 0
            while scan_idx < len(laser_scan.ranges):
                if laser_scan.ranges[scan_idx] < laser_scan.range_min\
                        or laser_scan.ranges[scan_idx] > laser_scan.range_max:
                    dist_diff.append(1)
                else:
                    dist_diff.append(laser_scan.ranges[scan_idx] / scan[scan_idx])
                scan_idx += 10

            dist_idx = 0
            dist_exp = 0
            while dist_idx < len(dist_diff):
                dist_exp += dist_diff[dist_idx]
                dist_idx += 1
            dist_exp = dist_exp / len(dist_diff)

            if dist_exp <= 0.8:
                self.probability = 0.4
            elif dist_exp < 0.95:
                self.probability = (4/3) * dist_exp - (2 /3)
            elif dist_exp <= 1.05:
                self.probability = 0.6
            elif dist_exp <= 1.2:
                self.probability = (-8 / 3) * dist_exp + 3.4
            else:
                self.probability = 0.2