import yaml
import numpy as np

# import scipy.linalg as linalg
import math


class LidarExtrinsic:
    def __init__(self, path_yaml):
        self.R_pandar_0 = []
        self.t_pandar_0 = []
        self.R_pandar_2 = []
        self.t_pandar_2 = []
        self.path_yaml = path_yaml
        self.load()

    # def rotate_mat(self, axis, radian):
    #     return linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))

    def to_RT(self, extrinsic_list):
        t = []
        R = []
        t.append(
            (
                extrinsic_list["delta_x"],
                extrinsic_list["delta_y"],
                extrinsic_list["delta_z"],
            )
        )
        t = np.array(t)

        yaw = extrinsic_list["heading"] / 180.0 * math.pi
        pitch = extrinsic_list["pitch"] / 180.0 * math.pi
        roll = extrinsic_list["roll"] / 180.0 * math.pi

        rot_z = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )
        rot_y = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )
        rot_x = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )
        R = np.dot(rot_z, np.dot(rot_y, rot_x))


        return R, t

    def load(self):
        with open(self.path_yaml, "r") as f:
            extrinsic_list = yaml.safe_load(f).get("extrinsic")
        # left
        self.R_pandar_0, self.t_pandar_0 = self.to_RT(extrinsic_list[0])
        # right
        self.R_pandar_2, self.t_pandar_2 = self.to_RT(extrinsic_list[1])
        # top
        # self.R_vlp16, self.t_vlp16 = self.to_RT(extrinsic_list[2])


if __name__ == "__main__":
    extrinsic = LidarExtrinsic("-.yaml")

    R = extrinsic.R_pandar_0
    print(R)
