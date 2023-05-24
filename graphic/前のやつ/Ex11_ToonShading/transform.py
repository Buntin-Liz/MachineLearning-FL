# -*- coding: utf-8 -*-
import numpy as np


def translate(tx, ty, tz):
  T = np.array([[1, 0, 0, tx],
                [0, 1, 0, ty],
                [0, 0, 1, tz],
                [0, 0, 0, 1]], dtype=float)
  return T


def scale(sx, sy, sz):
  S = np.array([[sx, 0, 0, 0],
                [0, sy, 0, 0],
                [0, 0, sz, 0],
                [0, 0, 0, 1]], dtype=float)
  return S


def rotate_x(theta):
  t = np.pi * theta / 180
  c, s = np.cos(t), np.sin(t)
  Rx = np.array([[1, 0, 0, 0],
                 [0, c, -s, 0],
                 [0, s, c, 0],
                 [0, 0, 0, 1]], dtype=float)
  return Rx


def rotate_y(theta):
  t = np.pi * theta / 180
  c, s = np.cos(t), np.sin(t)
  Ry = np.array([[c, 0, s, 0],
                 [0, 1, 0, 0],
                 [-s, 0, c, 0],
                 [0, 0, 0, 1]], dtype=float)
  return Ry


def rotate_z(theta):
  t = np.pi * theta / 180
  c, s = np.cos(t), np.sin(t)
  Rz = np.array([[c, -s, 0, 0],
                 [s, c, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]], dtype=float)
  return Rz
