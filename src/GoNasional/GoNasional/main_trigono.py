# distance.py

import math


class DistanceEstimator:

    def __init__(self):

        #####################################
        # Parameter Robot
        #####################################

        self.LH = 12.0

        # tinggi titik putar servo terhadap tanah
        self.H = 42.0

    def hitung(self,
               servo_head,
               vertical_bearing):

        """
        servo_head : radian
        vertical_bearing : radian

        return :
            dictionary
        """

        ###################################
        # Persamaan 3.1
        ###################################

        x = self.LH * math.cos(servo_head)

        ###################################
        # Persamaan 3.2
        ###################################

        y = self.LH * math.sin(servo_head)

        ###################################
        # Persamaan 3.3
        ###################################

        H_top = self.H + y

        ###################################
        # Persamaan 3.4
        ###################################

        beta_servo = (math.pi / 2) - servo_head

        ###################################
        # Sudut objek sebenarnya
        ###################################

        beta = beta_servo + vertical_bearing

        ###################################
        # Hindari pembagian nol
        ###################################

        if abs(math.tan(beta)) < 1e-6:

            return None

        ###################################
        # Persamaan 3.6
        ###################################

        D = H_top / math.tan(beta)

        ###################################
        # Persamaan 3.8
        ###################################

        D_actual = D + x

        return {

            "camera_x": x,
            "camera_y": y,
            "camera_height": H_top,

            "beta_servo": beta_servo,
            "beta_total": beta,

            "distance_camera": D,
            "distance_robot": D_actual

        }