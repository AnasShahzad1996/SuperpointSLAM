import os
import cv2
import math
import rclpy
import torch
import numpy as np


class GCNExtractor:
    def __init__(self, nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST):
        # Patch parameters
        self.PATCH_SIZE = 31
        self.HALF_PATCH_SIZE = 15
        self.EDGE_THRESHOLD = 19

        self.iniThFAST = None
        self.minThFAST = None

        self.scale_factor = scale_factor  # DONE
        self.n_levels = n_levels  # DONE
        self.n_features = n_features  # DONE

        self.mnFeaturesPerLevel = []
        self.umax = []

        self.mvScaleFactor = []
        self.mvInvScaleFactor = []
        self.mvLevelSigma2 = []
        self.mvInvLevelSigma2 = []

        self.mvImagePyramid = []
        self.module = None  # Placeholder for PyTorch model

    def initialize_gcn(self):
        print("### GCN Extractor ###")
        self.mvScaleFactor.append(1.0)
        self.mvLevelSigma2.append(1.0)

        for i in range(1, self.n_levels):
            self.mvScaleFactor.append(self.mvScaleFactor[i - 1] * self.scaleFactor)
            self.mvLevelSigma2.append(self.mvScaleFactor[i] * self.mvScaleFactor[i])

        for i in range(0, self.n_levels):
            self.mvInvScaleFactor.append(1 / self.mvScaleFactor[i])
            self.mvInvLevelSigma2.append(1 / self.mvLevelSigma2[i])

        factor = 1.0 / self.scaleFactor
        nDesiredFeaturesPerScale = (
            self.n_features * (1 - factor) / (1 - (factor ^ self.n_levels))
        )

        sumFeatures = 0
        mnFeaturesPerLevel = []
        for i in range(0, self.n_levels - 1):
            mnFeaturesPerLevel.append(round(nDesiredFeaturesPerScale))
            sumFeatures = sumFeatures + mnFeaturesPerLevel[i]
            nDesiredFeaturesPerScale = nDesiredFeaturesPerScale * factor

        mnFeaturesPerLevel[self.n_levels - 1] = max(n_features - sumFeatures, 0)
        v, v0 = 0, 0
        vmax = math.floor(self.HALF_PATCH_SIZE * math.sqrt(2.0) / 2 + 1)
        vmin = math.ceil(self.HALF_PATCH_SIZE * math.sqrt(2.0) / 2)
        hp2 = self.HALF_PATCH_SIZE ** 2
        for v in range(0, vmax):
            self.umax.append(round(math.sqrt(hp2 - (v * v))))

        v0 = 0
        for v in range(self.HALF_PATCH_SIZE, (vmin - 1), -1):
            while self.umax[v0] == self.umax[v0 + 1]:
                v0 = v0 + 1
            self.umax[v] = v0
            v0 = v0 + 1

        model_path = "/home/anas/ros2_ws/src/slam_bk/model/gcn2_640x480.pt"
