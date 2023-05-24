# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d


def loadOBJ(fliePath):
  numVertices = 0
  numUVs = 0
  numNormals = 0
  numFaces = 0
  V = []
  UV = []
  N = []

  F = []
  uvIDs = []
  nIDs = []
  for line in open(fliePath, "r"):
    vals = line.split()
    if len(vals) == 0:
      continue
    if vals[0] == "v":
      v = list(map(float, vals[1:4]))
      V.append(v)

      numVertices += 1
    if vals[0] == "vt":
      vt = list(map(float, vals[1:3]))
      UV.append(vt)
      numUVs += 1
    if vals[0] == "vn":
      vn = list(map(float, vals[1:4]))
      N.append(vn)
      numNormals += 1
    if vals[0] == "f":
      fvID = []
      uvID = []
      nvID = []
      for f in vals[1:]:
        w = f.split("/")
        if numVertices > 0:
          fvID.append(int(w[0]) - 1)
        if numUVs > 0:
          uvID.append(int(w[1]) - 1)
        if numNormals > 0:
          nvID.append(int(w[2]) - 1)
      F.append(fvID)
      uvIDs.append(uvID)
      nIDs.append(nvID)
      numFaces += 1

  mesh = Mesh(np.array(V), F, np.array(N), nIDs, UV, uvIDs)
  return mesh


class Mesh:
  def __init__(self, V, F, N=[], nIDs=[], UV=[], uvIDs=[]):
    V = 2.0 * (V - (V.max(0) + V.min(0)) / 2) / max(V.max(0) - V.min(0))
    self.V = V
    self.F = F
    self.N = N
    self.nIDs = nIDs
    self.UV = UV
    self.uvIDs = uvIDs

    self.N_f = None

  def numVertices(self):
    return len(self.V)

  def numFaces(self):
    return len(self.F)

  def numNormals(self):
    return len(self.N)

  def numUVs(self):
    return len(self.UV)

  def computeVertexNormals(self):
    N = self.N
    nIDs = self.nIDs

    N_v = np.zeros_like(self.V)

    for f, nvID in zip(self.F, nIDs):
      N_v[f, :] = N[nvID, :]
    self.N_v = N_v

  def computeFaceNormals(self):
    V = self.V
    V1 = np.array([V[f[1]] - V[f[0]] for f in self.F])
    V2 = np.array([V[f[2]] - V[f[0]] for f in self.F])
    N_f = np.cross(V1, V2)
    norm = np.linalg.norm(N_f, axis=1)

    N_f = (N_f.T / norm).T

    self.N_f = N_f

  def __str__(self):
    message = "numVertices: {0}\n".format(self.numVertices())
    message += "numFaces: {0}\n".format(self.numFaces())
    message += "numNormals: {0}\n".format(self.numNormals())
    message += "numUVs: {0}\n".format(self.numUVs())
    return message


def plot_mesh(ax, V, F, facecolor, edgecolor, alpha=1.0, linewidth=1.0):
  V_zxy = V[:, [2, 0, 1]]

  FVs = [V_zxy[f] for f in F]

  ax.add_collection3d(art3d.Poly3DCollection(
      FVs, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, linewidth=linewidth))

  v_min = V_zxy.min(0)
  v_max = V_zxy.max(0)

  return v_min, v_max


class MeshViewer:
  def __init__(self, fig, ax=None):
    if ax is None:
      self.ax = fig.add_subplot(1, 1, 1, projection='3d')
    else:
      self.ax = ax

    self.v_min = None
    self.v_max = None

  def addMesh(self, V, F, C, edgecolor, alpha=1.0, linewidth=1.0):
    ax = self.ax

    v_min, v_max = plot_mesh(
        ax, V, F, C, edgecolor, alpha=alpha, linewidth=linewidth)

    if self.v_min is None:
      self.v_min = v_min
      self.v_max = v_max
    else:
      self.v_min = np.vstack((self.v_min, v_min)).min(0)
      self.v_max = np.vstack((self.v_max, v_max)).max(0)

  def setView(self, elev, azim):
    ax = self.ax
    ax.view_init(elev=elev, azim=azim)

  def show(self):
    ax = self.ax

    v_min = self.v_min
    v_max = self.v_max

    v_center = 0.5 * (v_min + v_max)
    v_d = 0.5 * np.mean(v_max - v_min)
    v_min = v_center - v_d
    v_max = v_center + v_d

    ax.set_xlim(v_min[0], v_max[0])
    ax.set_ylim(v_min[1], v_max[1])
    ax.set_zlim(v_min[2], v_max[2])

    ax.set_xlabel('Z')
    ax.set_ylabel('X')
    ax.set_zlabel('Y')
