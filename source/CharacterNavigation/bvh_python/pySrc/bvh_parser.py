import torch
import BVH_mod as BVH
import numpy as np
from Quaternions import Quaternions
from Kinematics import ForwardKinematics
from bvh_writer import write_bvh

class BVH_file:
    def __init__(self, filePath, jointNames = None):
        self.anim, self._names, self.frametime = BVH.load(filePath)
        self.edges = [] #incluye offsets
        self.edge_mat = []
        self.edge_num = 0
        self._topology = None
        self.ee_length = []
        if jointNames:
            self.subsetNames = jointNames
        else:
            self.subsetNames = self._names

        for i, name in enumerate(self._names):
            if ':' in name:
                name = name[name.find(':') + 1:]
                self._names[i] = name


        self.details = [i for i, name in enumerate(self._names) if name not in self.subsetNames]
        self.joint_num = self.anim.shape[1]
        self.joints = []
        self.simplified_name = []
        self.simplify_map = {}
        self.inverse_simplify_map = {}

        for name in self.subsetNames:
            for j in range(self.anim.shape[1]):
                if name == self._names[j]:
                    self.joints.append(j)
                    break

        if len(self.joints) != len(self.subsetNames):
            for i in self.joints: print(self._names[i], end=' ')
            print(self.joints, len(self.joints), sep='\n')
            raise Exception('Problem in file', filePath)

        self.joint_num_simplify = len(self.joints)
        for i, j in enumerate(self.joints):
            self.simplify_map[j] = i
            self.inverse_simplify_map[i] = j
            self.simplified_name.append(self._names[j])
        self.inverse_simplify_map[0] = -1
        for i in range(self.anim.shape[1]):
            if i in self.details:
                self.simplify_map[i] = -1


    def scale(self, alpha):
        self.anim.offsets *= alpha
        global_position = self.anim.positions[:, 0, :]
        global_position[1:, :] *= alpha
        global_position[1:, :] += (1 - alpha) * global_position[0, :]

    def rotate(self, theta, axis):
        q = Quaternions(np.hstack((np.cos(theta/2), np.sin(theta/2) * axis)))
        position = self.anim.positions[:, 0, :].copy()
        rotation = self.anim.rotations[:, 0, :]
        position[1:, ...] -= position[0:-1, ...]
        q_position = Quaternions(np.hstack((np.zeros((position.shape[0], 1)), position)))
        q_rotation = Quaternions.from_euler(np.radians(rotation))
        q_rotation = q * q_rotation
        q_position = q * q_position * (-q)
        self.anim.rotations[:, 0, :] = np.degrees(q_rotation.euler())
        position = q_position.imaginaries
        for i in range(1, position.shape[0]):
            position[i] += position[i-1]
        self.anim.positions[:, 0, :] = position

    @property
    def topology(self):
        if self._topology is None:
            # se extraen los padres de las joints filtradas
            self._topology = self.anim.parents[self.joints].copy()
            for i in range(self._topology.shape[0]):
                # con esto el id del padre referencia la posicion de la joint hija en self.corps
                # la raiz mantiene su padre con valor -1
                if i >= 1: self._topology[i] = self.simplify_map[self._topology[i]]
            self._topology = tuple(self._topology)
        return self._topology

    def to_numpy(self, quater=False, edge=True):
        rotations = self.anim.rotations[:, self.joints, :]
        if quater:
            rotations = Quaternions.from_euler(np.radians(rotations)).qs
            positions = self.anim.positions[:, 0, :]
        else:
            positions = self.anim.positions[:, 0, :]
        if edge:
            index = []
            for e in self.edges:
                index.append(e[0])
            rotations = rotations[:, index, :]

        # se concatenan las rotaciones de las articulaciones por frame
        rotations = rotations.reshape(rotations.shape[0], -1)
        # si edge=True no incluye rotacion para la raiz
        # al final de cada arreglo de rotaciones por frame se concatena la posicion de la raiz
        # [ frame1-> [rotXjoint1, rotYjoint1, rotZjoint1, rotXjoint2, rotYjoint2, rotZjoint2, ..., posX, posY, posZ], ..., frameN ->...]
        return np.concatenate((rotations, positions), axis=1)

    def to_tensor(self, quater=False, edge=True):
        res = self.to_numpy(quater, edge)
        res = torch.tensor(res, dtype=torch.float)
        res = res.permute(1, 0)
        res = res.reshape((-1, res.shape[-1]))
        # si edge=True no incluye rotacion para la raiz
        # la forma final del tensor es la de self.to_numpy con las dimensiones 0 y 1 invertidas. Se recupera la info de un frame k, 
        # tomando el k-ésimo elemento de cada fila.
        return res

    def get_positions(self):
        positions = self.anim.positions
        positions = positions[:, self.joints, :]
        return positions

    @property
    def offsets(self): #offsets incluye la raiz
        return self.anim.offsets[self.joints]

    @property
    def names(self):
        return self.simplified_name

    def write(self, file_path):
        motion = self.to_numpy(quater=False, edge=False)
        rotations = motion[..., :-3].reshape(motion.shape[0], -1, 3)
        positions = motion[..., -3:]
        write_bvh(self.topology, self.offsets, rotations, positions, self.names, 1.0/30, 'xyz', file_path)

    def set_new_root(self, new_root):
        euler = torch.tensor(self.anim.rotations[:, 0, :], dtype=torch.float)
        transform = ForwardKinematics.transform_from_euler(euler, 'xyz')
        offset = torch.tensor(self.anim.offsets[new_root], dtype=torch.float)
        new_pos = torch.matmul(transform, offset)
        new_pos = new_pos.numpy() + self.anim.positions[:, 0, :]
        self.anim.offsets[0] = -self.anim.offsets[new_root]
        self.anim.offsets[new_root] = np.zeros((3, ))
        self.anim.positions[:, new_root, :] = new_pos
        rot0 = Quaternions.from_euler(np.radians(self.anim.rotations[:, 0, :]), order='xyz')
        rot1 = Quaternions.from_euler(np.radians(self.anim.rotations[:, new_root, :]), order='xyz')
        new_rot1 = rot0 * rot1
        new_rot0 = (-rot1)
        new_rot0 = np.degrees(new_rot0.euler())
        new_rot1 = np.degrees(new_rot1.euler())
        self.anim.rotations[:, 0, :] = new_rot0
        self.anim.rotations[:, new_root, :] = new_rot1

        new_seq = []
        vis = [0] * self.anim.rotations.shape[1]
        new_idx = [-1] * len(vis)
        new_parent = [0] * len(vis)

        def relabel(x):
            nonlocal new_seq, vis, new_idx, new_parent
            new_idx[x] = len(new_seq)
            new_seq.append(x)
            vis[x] = 1
            for y in range(len(vis)):
                if not vis[y] and (self.anim.parents[x] == y or self.anim.parents[y] == x):
                    relabel(y)
                    new_parent[new_idx[y]] = new_idx[x]

        relabel(new_root)
        self.anim.rotations = self.anim.rotations[:, new_seq, :]
        self.anim.offsets = self.anim.offsets[new_seq]
        names = self._names.copy()
        for i, j in enumerate(new_seq):
            self._names[i] = names[j]
        self.anim.parents = np.array(new_parent, dtype=np.int)

