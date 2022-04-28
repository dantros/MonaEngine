import operator
import re
import numpy as np
import torch.nn as nn
import torch
import math


class Animation:
    """
    Animation is a numpy-like wrapper for animation data
    
    Animation data consists of several arrays consisting
    of F frames and J joints.
    
    The animation is specified by
    
        rotations : (F, J) Quaternions | Joint Rotations
        positions : (F, J, 3) ndarray  | Joint Positions
    
    The base pose is specified by
    
        orients   : (J) Quaternions    | Joint Orientations
        offsets   : (J, 3) ndarray     | Joint Offsets
        
    And the skeletal structure is specified by
        
        parents   : (J) ndarray        | Joint Parents
    """
    
    def __init__(self, rotations, positions, orients, offsets, parents):
        
        self.rotations = rotations
        self.positions = positions
        self.orients   = orients
        self.offsets   = offsets
        self.parents   = parents
    
    def __op__(self, op, other):
        return Animation(
            op(self.rotations, other.rotations),
            op(self.positions, other.positions),
            op(self.orients, other.orients),
            op(self.offsets, other.offsets),
            op(self.parents, other.parents))

    def __iop__(self, op, other):
        self.rotations = op(self.roations, other.rotations)
        self.positions = op(self.roations, other.positions)
        self.orients   = op(self.orients, other.orients)
        self.offsets   = op(self.offsets, other.offsets)
        self.parents   = op(self.parents, other.parents)
        return self
    
    def __sop__(self, op):
        return Animation(
            op(self.rotations),
            op(self.positions),
            op(self.orients),
            op(self.offsets),
            op(self.parents))
    
    def __add__(self, other): return self.__op__(operator.add, other)
    def __sub__(self, other): return self.__op__(operator.sub, other)
    def __mul__(self, other): return self.__op__(operator.mul, other)
    def __div__(self, other): return self.__op__(operator.div, other)
    
    def __abs__(self): return self.__sop__(operator.abs)
    def __neg__(self): return self.__sop__(operator.neg)
    
    def __iadd__(self, other): return self.__iop__(operator.iadd, other)
    def __isub__(self, other): return self.__iop__(operator.isub, other)
    def __imul__(self, other): return self.__iop__(operator.imul, other)
    def __idiv__(self, other): return self.__iop__(operator.idiv, other)
    
    def __len__(self): return len(self.rotations)
    
    def __getitem__(self, k):
        if isinstance(k, tuple):
            return Animation(
                self.rotations[k],
                self.positions[k],
                self.orients[k[1:]],
                self.offsets[k[1:]],
                self.parents[k[1:]]) 
        else:
            return Animation(
                self.rotations[k],
                self.positions[k],
                self.orients,
                self.offsets,
                self.parents) 
        
    def __setitem__(self, k, v): 
        if isinstance(k, tuple):
            self.rotations.__setitem__(k, v.rotations)
            self.positions.__setitem__(k, v.positions)
            self.orients.__setitem__(k[1:], v.orients)
            self.offsets.__setitem__(k[1:], v.offsets)
            self.parents.__setitem__(k[1:], v.parents)
        else:
            self.rotations.__setitem__(k, v.rotations)
            self.positions.__setitem__(k, v.positions)
            self.orients.__setitem__(k, v.orients)
            self.offsets.__setitem__(k, v.offsets)
            self.parents.__setitem__(k, v.parents)
        
    @property
    def shape(self): return (self.rotations.shape[0], self.rotations.shape[1])
            
    def copy(self): return Animation(
        self.rotations.copy(), self.positions.copy(), 
        self.orients.copy(), self.offsets.copy(), 
        self.parents.copy())
    
    def repeat(self, *args, **kw):
        return Animation(
            self.rotations.repeat(*args, **kw),
            self.positions.repeat(*args, **kw),
            self.orients, self.offsets, self.parents)   



class BVH:
    channelmap = {
    'Xrotation' : 'x',
    'Yrotation' : 'y',
    'Zrotation' : 'z'   
    }

    channelmap_inv = {
        'x': 'Xrotation',
        'y': 'Yrotation',
        'z': 'Zrotation',
    }

    ordermap = {
        'x' : 0,
        'y' : 1,
        'z' : 2,
    }

    def load(self, filename, start=None, end=None, order=None, world=False, need_quater=False):
        """
        Reads a BVH file and constructs an animation
        
        Parameters
        ----------
        filename: str
            File to be opened
            
        start : int
            Optional Starting Frame
            
        end : int
            Optional Ending Frame
        
        order : str
            Optional Specifier for joint order.
            Given as string E.G 'xyz', 'zxy'
            
        world : bool
            If set to true euler angles are applied
            together in world space rather than local
            space

        Returns
        -------
        
        (animation, joint_names, frametime)
            Tuple of loaded animation and joint names
        """
        
        f = open(filename, "r")

        i = 0
        active = -1
        end_site = False
        
        names = []
        orients = Quaternions.id(0)
        offsets = np.array([]).reshape((0,3))
        parents = np.array([], dtype=int)
        
        for line in f:
            
            if "HIERARCHY" in line: continue
            if "MOTION" in line: continue

            """ Modified line read to handle Mixamo data """
    #        rmatch = re.match(r"ROOT (\w+)", line)
            rmatch = re.match(r"ROOT (\w+:?\w+)", line)
            if rmatch:
                names.append(rmatch.group(1))
                offsets    = np.append(offsets,    np.array([[0,0,0]]),   axis=0)
                orients.qs = np.append(orients.qs, np.array([[1,0,0,0]]), axis=0)
                parents    = np.append(parents, active)
                active = (len(parents)-1)
                continue

            if "{" in line: continue

            if "}" in line:
                if end_site: end_site = False
                else: active = parents[active]
                continue
            
            offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
            if offmatch:
                if not end_site:
                    offsets[active] = np.array([list(map(float, offmatch.groups()))])
                continue
            
            chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
            if chanmatch:
                channels = int(chanmatch.group(1))
                if order is None:
                    channelis = 0 if channels == 3 else 3
                    channelie = 3 if channels == 3 else 6
                    parts = line.split()[2+channelis:2+channelie]
                    if any([p not in self.channelmap for p in parts]):
                        continue
                    order = "".join([self.channelmap[p] for p in parts])
                continue

            """ Modified line read to handle Mixamo data """
    #        jmatch = re.match("\s*JOINT\s+(\w+)", line)
            jmatch = re.match("\s*JOINT\s+(\w+:?\w+)", line)
            if jmatch:
                names.append(jmatch.group(1))
                offsets    = np.append(offsets,    np.array([[0,0,0]]),   axis=0)
                orients.qs = np.append(orients.qs, np.array([[1,0,0,0]]), axis=0)
                parents    = np.append(parents, active)
                active = (len(parents)-1)
                continue
            
            if "End Site" in line:
                end_site = True
                continue
                
            fmatch = re.match("\s*Frames:\s+(\d+)", line)
            if fmatch:
                if start and end:
                    fnum = (end - start)-1
                else:
                    fnum = int(fmatch.group(1))
                jnum = len(parents)
                positions = offsets[np.newaxis].repeat(fnum, axis=0)
                rotations = np.zeros((fnum, len(orients), 3))
                continue
            
            fmatch = re.match("\s*Frame Time:\s+([\d\.]+)", line)
            if fmatch:
                frametime = float(fmatch.group(1))
                continue
            
            if (start and end) and (i < start or i >= end-1):
                i += 1
                continue
            
            dmatch = line.strip().split()
            if dmatch:
                data_block = np.array(list(map(float, dmatch)))
                N = len(parents)
                fi = i - start if start else i
                if   channels == 3:
                    positions[fi,0:1] = data_block[0:3]
                    rotations[fi, : ] = data_block[3: ].reshape(N,3)
                elif channels == 6:
                    data_block = data_block.reshape(N,6)
                    positions[fi,:] = data_block[:,0:3]
                    rotations[fi,:] = data_block[:,3:6]
                elif channels == 9:
                    positions[fi,0] = data_block[0:3]
                    data_block = data_block[3:].reshape(N-1,9)
                    rotations[fi,1:] = data_block[:,3:6]
                    positions[fi,1:] += data_block[:,0:3] * data_block[:,6:9]
                else:
                    raise Exception("Too many channels! %i" % channels)

                i += 1

        f.close()

        if need_quater:
            rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
        elif order != 'xyz':
            rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
            rotations = np.degrees(rotations.euler())
        
        return (Animation(rotations, positions, orients, offsets, parents), names, frametime)
        

        
    def save(self, filename, anim, names=None, frametime=1.0/24.0, order='zyx', positions=False, mask=None, quater=False):
        """
        Saves an Animation to file as BVH
        
        Parameters
        ----------
        filename: str
            File to be saved to
            
        anim : Animation
            Animation to save
            
        names : [str]
            List of joint names
        
        order : str
            Optional Specifier for joint order.
            Given as string E.G 'xyz', 'zxy'
        
        frametime : float
            Optional Animation Frame time
            
        positions : bool
            Optional specfier to save bone
            positions for each frame
            
        orients : bool
            Multiply joint orients to the rotations
            before saving.
            
        """
        
        if names is None:
            names = ["joint_" + str(i) for i in range(len(anim.parents))]
        
        with open(filename, 'w') as f:

            t = ""
            f.write("%sHIERARCHY\n" % t)
            f.write("%sROOT %s\n" % (t, names[0]))
            f.write("%s{\n" % t)
            t += '\t'

            f.write("%sOFFSET %f %f %f\n" % (t, anim.offsets[0,0], anim.offsets[0,1], anim.offsets[0,2]) )
            f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % 
                (t, self.channelmap_inv[order[0]], self.channelmap_inv[order[1]], self.channelmap_inv[order[2]]))

            for i in range(anim.shape[1]):
                if anim.parents[i] == 0:
                    t = self.save_joint(f, anim, names, t, i, order=order, positions=positions)

            t = t[:-1]
            f.write("%s}\n" % t)

            f.write("MOTION\n")
            f.write("Frames: %i\n" % anim.shape[0])
            f.write("Frame Time: %f\n" % frametime)
                
            if quater:
                rots = np.degrees(anim.rotations.euler(order=order[::-1]))
            else:
                rots = anim.rotations
            poss = anim.positions
            
            for i in range(anim.shape[0]):
                for j in range(anim.shape[1]):
                    
                    if positions or j == 0:
                    
                        f.write("%f %f %f %f %f %f " % (
                            poss[i,j,0],                  poss[i,j,1],                  poss[i,j,2], 
                            rots[i,j,self.ordermap[order[0]]], rots[i,j,self.ordermap[order[1]]], rots[i,j,self.ordermap[order[2]]]))
                    
                    else:
                        if mask == None or mask[j] == 1:
                            f.write("%f %f %f " % (
                                rots[i,j,self.ordermap[order[0]]], rots[i,j,self.ordermap[order[1]]], rots[i,j,self.ordermap[order[2]]]))
                        else:
                            f.write("%f %f %f " % (0, 0, 0))

                f.write("\n")
        
        
    def save_joint(self, f, anim, names, t, i, order='zyx', positions=False):
        
        f.write("%sJOINT %s\n" % (t, names[i]))
        f.write("%s{\n" % t)
        t += '\t'
    
        f.write("%sOFFSET %f %f %f\n" % (t, anim.offsets[i,0], anim.offsets[i,1], anim.offsets[i,2]))
        
        if positions:
            f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % (t, 
                self.channelmap_inv[order[0]], self.channelmap_inv[order[1]], self.channelmap_inv[order[2]]))
        else:
            f.write("%sCHANNELS 3 %s %s %s\n" % (t, 
                self.channelmap_inv[order[0]], self.channelmap_inv[order[1]], self.channelmap_inv[order[2]]))
        
        end_site = True
        
        for j in range(anim.shape[1]):
            if anim.parents[j] == i:
                t = self.save_joint(f, anim, names, t, j, order=order, positions=positions)
                end_site = False
        
        if end_site:
            f.write("%sEnd Site\n" % t)
            f.write("%s{\n" % t)
            t += '\t'
            f.write("%sOFFSET %f %f %f\n" % (t, 0.0, 0.0, 0.0))
            t = t[:-1]
            f.write("%s}\n" % t)
    
        t = t[:-1]
        f.write("%s}\n" % t)
        
        return t


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
        BVH_writer.write_bvh(self.topology, self.offsets, rotations, positions, self.names, 1.0/30, 'xyz', file_path)

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


class BVH_writer():
    def __init__(self, stdbvhPath):
        staticFile = BVH_file(stdbvhPath)
        self.parent = staticFile.topology
        self.offset = staticFile.offsets
        self.names = staticFile.names
        self.joint_num = len(self.parent)

    # position, rotation with shape T * J * 3
    def write(self, rotations, positions, path, frametime=1.0/30):
        order = 'xyz'
        
        if rotations.shape[1] < self.joint_num:
            rootRot = torch.zeros((rotations.shape[0], 1, 3))
            rotations = torch.cat((rootRot, rotations), 1)

        return self.write_bvh(self.parent, self.offset, rotations, positions, self.names, frametime, order, path)

    def write_raw(self, motion, order, path, frametime=1.0/30):
        motion = motion.permute(1, 0).detach().cpu().numpy()
        positions = motion[:, -3:]
        rotations = motion[:, :-3]
        if order == 'quaternion':
            rotations = rotations.reshape((motion.shape[0], -1, 4))
        else:
            rotations = rotations.reshape((motion.shape[0], -1, 3))

        return self.write(rotations, positions, order, path, frametime)
    
    # rotation with shape frame * J * 3
    @staticmethod
    def write_bvh(parent, offset, rotation, position, names, frametime, order, path, endsite=None): #offset, rotations,.. incluye la raiz 
        file = open(path, 'w')
        frame = rotation.shape[0]
        joint_num = rotation.shape[1]
        order = order.upper()

        file_string = 'HIERARCHY\n'

        def write_static(idx, prefix):
            nonlocal parent, offset, rotation, names, order, endsite, file_string
            if idx == 0:
                name_label = 'ROOT ' + names[idx]
                channel_label = 'CHANNELS 6 Xposition Yposition Zposition {}rotation {}rotation {}rotation'.format(*order)
            else:
                name_label = 'JOINT ' + names[idx]
                channel_label = 'CHANNELS 3 {}rotation {}rotation {}rotation'.format(*order)
            offset_label = 'OFFSET %.6f %.6f %.6f' % (offset[idx][0], offset[idx][1], offset[idx][2])

            file_string += prefix + name_label + '\n'
            file_string += prefix + '{\n'
            file_string += prefix + '\t' + offset_label + '\n'
            file_string += prefix + '\t' + channel_label + '\n'

            has_child = False
            for y in range(idx+1, rotation.shape[1]):
                if parent[y] == idx:
                    has_child = True
                    write_static(y, prefix + '\t')
            if not has_child:
                file_string += prefix + '\t' + 'End Site\n'
                file_string += prefix + '\t' + '{\n'
                file_string += prefix + '\t\t' + 'OFFSET 0 0 0\n'
                file_string += prefix + '\t' + '}\n'

            file_string += prefix + '}\n'

        write_static(0, '')

        file_string += 'MOTION\n' + 'Frames: {}\n'.format(frame) + 'Frame Time: %.8f\n' % frametime
        for i in range(frame):
            file_string += '%.6f %.6f %.6f ' % (position[i][0], position[i][1], position[i][2])
            for j in range(joint_num):
                file_string += '%.6f %.6f %.6f ' % (rotation[i][j][0], rotation[i][j][1], rotation[i][j][2])
            file_string += '\n'

        file.write(file_string)
        return file_string


class ForwardKinematics:
    def __init__(self, args, edges):
        self.topology = [-1] * (len(edges) + 1)
        self.rotation_map = []
        for i, edge in enumerate(edges):
            self.topology[edge[1]] = edge[0]
            self.rotation_map.append(edge[1])

        self.world = args.fk_world
        self.pos_repr = args.pos_repr
        self.quater = args.rotation == 'quaternion'

    def forward_from_raw(self, raw, offset, world=None, quater=None):
        if world is None: world = self.world
        if quater is None: quater = self.quater
        if self.pos_repr == '3d':
            position = raw[:, -3:, :]
            rotation = raw[:, :-3, :]
        elif self.pos_repr == '4d':
            raise Exception('Not support')
        if quater:
            rotation = rotation.reshape((rotation.shape[0], -1, 4, rotation.shape[-1]))
            identity = torch.tensor((1, 0, 0, 0), dtype=torch.float, device=raw.device)
        else:
            rotation = rotation.reshape((rotation.shape[0], -1, 3, rotation.shape[-1]))
            identity = torch.zeros((3, ), dtype=torch.float, device=raw.device)
        identity = identity.reshape((1, 1, -1, 1))
        new_shape = list(rotation.shape)
        new_shape[1] += 1
        new_shape[2] = 1
        rotation_final = identity.repeat(new_shape)
        for i, j in enumerate(self.rotation_map):
            rotation_final[:, j, :, :] = rotation[:, i, :, :]
        return self.forward(rotation_final, position, offset, world=world, quater=quater)

    '''
    rotation should have shape batch_size * Joint_num * (3/4) * Time
    position should have shape batch_size * 3 * Time
    offset should have shape batch_size * Joint_num * 3
    output have shape batch_size * Time * Joint_num * 3
    '''
    def forward(self, rotation: torch.Tensor, position: torch.Tensor, offset: torch.Tensor, order='xyz', quater=False, world=True):
        if not quater and rotation.shape[-2] != 3: raise Exception('Unexpected shape of rotation')
        if quater and rotation.shape[-2] != 4: raise Exception('Unexpected shape of rotation')
        rotation = rotation.permute(0, 3, 1, 2)
        position = position.permute(0, 2, 1)
        result = torch.empty(rotation.shape[:-1] + (3, ), device=position.device)


        norm = torch.norm(rotation, dim=-1, keepdim=True)
        #norm[norm < 1e-10] = 1
        rotation = rotation / norm


        if quater:
            transform = self.transform_from_quaternion(rotation)
        else:
            transform = self.transform_from_euler(rotation, order)

        offset = offset.reshape((-1, 1, offset.shape[-2], offset.shape[-1], 1))

        result[..., 0, :] = position
        for i, pi in enumerate(self.topology):
            if pi == -1:
                assert i == 0
                continue

            transform[..., i, :, :] = torch.matmul(transform[..., pi, :, :].clone(), transform[..., i, :, :].clone())
            result[..., i, :] = torch.matmul(transform[..., i, :, :], offset[..., i, :, :]).squeeze()
            if world: result[..., i, :] += result[..., pi, :]
        return result

    def from_local_to_world(self, res: torch.Tensor):
        res = res.clone()
        for i, pi in enumerate(self.topology):
            if pi == 0 or pi == -1:
                continue
            res[..., i, :] += res[..., pi, :]
        return res

    @staticmethod
    def transform_from_euler(rotation, order):
        rotation = rotation / 180 * math.pi
        transform = torch.matmul(ForwardKinematics.transform_from_axis(rotation[..., 1], order[1]),
                                 ForwardKinematics.transform_from_axis(rotation[..., 2], order[2]))
        transform = torch.matmul(ForwardKinematics.transform_from_axis(rotation[..., 0], order[0]), transform)
        return transform

    @staticmethod
    def transform_from_axis(euler, axis):
        transform = torch.empty(euler.shape[0:3] + (3, 3), device=euler.device)
        cos = torch.cos(euler)
        sin = torch.sin(euler)
        cord = ord(axis) - ord('x')

        transform[..., cord, :] = transform[..., :, cord] = 0
        transform[..., cord, cord] = 1

        if axis == 'x':
            transform[..., 1, 1] = transform[..., 2, 2] = cos
            transform[..., 1, 2] = -sin
            transform[..., 2, 1] = sin
        if axis == 'y':
            transform[..., 0, 0] = transform[..., 2, 2] = cos
            transform[..., 0, 2] = sin
            transform[..., 2, 0] = -sin
        if axis == 'z':
            transform[..., 0, 0] = transform[..., 1, 1] = cos
            transform[..., 0, 1] = -sin
            transform[..., 1, 0] = sin

        return transform

    @staticmethod
    def transform_from_quaternion(quater: torch.Tensor):
        qw = quater[..., 0]
        qx = quater[..., 1]
        qy = quater[..., 2]
        qz = quater[..., 3]

        x2 = qx + qx
        y2 = qy + qy
        z2 = qz + qz
        xx = qx * x2
        yy = qy * y2
        wx = qw * x2
        xy = qx * y2
        yz = qy * z2
        wy = qw * y2
        xz = qx * z2
        zz = qz * z2
        wz = qw * z2

        m = torch.empty(quater.shape[:-1] + (3, 3), device=quater.device)
        m[..., 0, 0] = 1.0 - (yy + zz)
        m[..., 0, 1] = xy - wz
        m[..., 0, 2] = xz + wy
        m[..., 1, 0] = xy + wz
        m[..., 1, 1] = 1.0 - (xx + zz)
        m[..., 1, 2] = yz - wx
        m[..., 2, 0] = xz - wy
        m[..., 2, 1] = yz + wx
        m[..., 2, 2] = 1.0 - (xx + yy)

        return m


class InverseKinematics:
    def __init__(self, rotations: torch.Tensor, positions: torch.Tensor, offset, parents, constrains):
        self.rotations = rotations
        self.rotations.requires_grad_(True)
        self.position = positions
        self.position.requires_grad_(True)

        self.parents = parents
        self.offset = offset
        self.constrains = constrains

        self.optimizer = torch.optim.Adam([self.position, self.rotations], lr=1e-3, betas=(0.9, 0.999))
        self.crit = nn.MSELoss()

    def step(self):
        self.optimizer.zero_grad()
        glb = self.forward(self.rotations, self.position, self.offset, order='', quater=True, world=True)
        loss = self.crit(glb, self.constrains)
        loss.backward()
        self.optimizer.step()
        self.glb = glb
        return loss.item()

    def tloss(self, time):
        return self.crit(self.glb[time, :], self.constrains[time, :])

    def all_loss(self):
        res = [self.tloss(t).detach().numpy() for t in range(self.constrains.shape[0])]
        return np.array(res)

    '''
        rotation should have shape batch_size * Joint_num * (3/4) * Time
        position should have shape batch_size * 3 * Time
        offset should have shape batch_size * Joint_num * 3
        output have shape batch_size * Time * Joint_num * 3
    '''

    def forward(self, rotation: torch.Tensor, position: torch.Tensor, offset: torch.Tensor, order='xyz', quater=False,
                world=True):
        '''
        if not quater and rotation.shape[-2] != 3: raise Exception('Unexpected shape of rotation')
        if quater and rotation.shape[-2] != 4: raise Exception('Unexpected shape of rotation')
        rotation = rotation.permute(0, 3, 1, 2)
        position = position.permute(0, 2, 1)
        '''
        result = torch.empty(rotation.shape[:-1] + (3,), device=position.device)

        norm = torch.norm(rotation, dim=-1, keepdim=True)
        rotation = rotation / norm

        if quater:
            transform = self.transform_from_quaternion(rotation)
        else:
            transform = self.transform_from_euler(rotation, order)

        offset = offset.reshape((-1, 1, offset.shape[-2], offset.shape[-1], 1))

        result[..., 0, :] = position
        for i, pi in enumerate(self.parents):
            if pi == -1:
                assert i == 0
                continue

            result[..., i, :] = torch.matmul(transform[..., pi, :, :], offset[..., i, :, :]).squeeze()
            transform[..., i, :, :] = torch.matmul(transform[..., pi, :, :].clone(), transform[..., i, :, :].clone())
            if world: result[..., i, :] += result[..., pi, :]
        return result

    @staticmethod
    def transform_from_euler(rotation, order):
        rotation = rotation / 180 * math.pi
        transform = torch.matmul(ForwardKinematics.transform_from_axis(rotation[..., 1], order[1]),
                                 ForwardKinematics.transform_from_axis(rotation[..., 2], order[2]))
        transform = torch.matmul(ForwardKinematics.transform_from_axis(rotation[..., 0], order[0]), transform)
        return transform

    @staticmethod
    def transform_from_axis(euler, axis):
        transform = torch.empty(euler.shape[0:3] + (3, 3), device=euler.device)
        cos = torch.cos(euler)
        sin = torch.sin(euler)
        cord = ord(axis) - ord('x')

        transform[..., cord, :] = transform[..., :, cord] = 0
        transform[..., cord, cord] = 1

        if axis == 'x':
            transform[..., 1, 1] = transform[..., 2, 2] = cos
            transform[..., 1, 2] = -sin
            transform[..., 2, 1] = sin
        if axis == 'y':
            transform[..., 0, 0] = transform[..., 2, 2] = cos
            transform[..., 0, 2] = sin
            transform[..., 2, 0] = -sin
        if axis == 'z':
            transform[..., 0, 0] = transform[..., 1, 1] = cos
            transform[..., 0, 1] = -sin
            transform[..., 1, 0] = sin

        return transform

    @staticmethod
    def transform_from_quaternion(quater: torch.Tensor):
        qw = quater[..., 0]
        qx = quater[..., 1]
        qy = quater[..., 2]
        qz = quater[..., 3]

        x2 = qx + qx
        y2 = qy + qy
        z2 = qz + qz
        xx = qx * x2
        yy = qy * y2
        wx = qw * x2
        xy = qx * y2
        yz = qy * z2
        wy = qw * y2
        xz = qx * z2
        zz = qz * z2
        wz = qw * z2

        m = torch.empty(quater.shape[:-1] + (3, 3), device=quater.device)
        m[..., 0, 0] = 1.0 - (yy + zz)
        m[..., 0, 1] = xy - wz
        m[..., 0, 2] = xz + wy
        m[..., 1, 0] = xy + wz
        m[..., 1, 1] = 1.0 - (xx + zz)
        m[..., 1, 2] = yz - wx
        m[..., 2, 0] = xz - wy
        m[..., 2, 1] = yz + wx
        m[..., 2, 2] = 1.0 - (xx + yy)

        return m




class Quaternions:
    """
    Quaternions is a wrapper around a numpy ndarray
    that allows it to act as if it were an narray of
    a quater data type.
    
    Therefore addition, subtraction, multiplication,
    division, negation, absolute, are all defined
    in terms of quater operations such as quater
    multiplication.
    
    This allows for much neater code and many routines
    which conceptually do the same thing to be written
    in the same way for point data and for rotation data.
    
    The Quaternions class has been desgined such that it
    should support broadcasting and slicing in all of the
    usual ways.
    """
    
    def __init__(self, qs):
        if isinstance(qs, np.ndarray):
            if len(qs.shape) == 1: qs = np.array([qs])
            self.qs = qs
            return

        if isinstance(qs, Quaternions):
            self.qs = qs
            return

        raise TypeError('Quaternions must be constructed from iterable, numpy array, or Quaternions, not %s' % type(qs))
    
    def __str__(self): return "Quaternions("+ str(self.qs) + ")"
    def __repr__(self): return "Quaternions("+ repr(self.qs) + ")"
    
    """ Helper Methods for Broadcasting and Data extraction """
    
    @classmethod
    def _broadcast(cls, sqs, oqs, scalar=False):
        if isinstance(oqs, float): return sqs, oqs * np.ones(sqs.shape[:-1])
        
        ss = np.array(sqs.shape) if not scalar else np.array(sqs.shape[:-1])
        os = np.array(oqs.shape)

        if len(ss) != len(os):
            raise TypeError('Quaternions cannot broadcast together shapes %s and %s' % (sqs.shape, oqs.shape))
            
        if np.all(ss == os): return sqs, oqs
        
        if not np.all((ss == os) | (os == np.ones(len(os))) | (ss == np.ones(len(ss)))):
            raise TypeError('Quaternions cannot broadcast together shapes %s and %s' % (sqs.shape, oqs.shape))

        sqsn, oqsn = sqs.copy(), oqs.copy()

        for a in np.where(ss == 1)[0]: sqsn = sqsn.repeat(os[a], axis=a)
        for a in np.where(os == 1)[0]: oqsn = oqsn.repeat(ss[a], axis=a)
        
        return sqsn, oqsn
        
    """ Adding Quaterions is just Defined as Multiplication """
    
    def __add__(self, other): return self * other
    def __sub__(self, other): return self / other
    
    """ Quaterion Multiplication """
    
    def __mul__(self, other):
        """
        Quaternion multiplication has three main methods.
        
        When multiplying a Quaternions array by Quaternions
        normal quater multiplication is performed.
        
        When multiplying a Quaternions array by a vector
        array of the same shape, where the last axis is 3,
        it is assumed to be a Quaternion by 3D-Vector 
        multiplication and the 3D-Vectors are rotated
        in space by the Quaternions.
        
        When multipplying a Quaternions array by a scalar
        or vector of different shape it is assumed to be
        a Quaternions by Scalars multiplication and the
        Quaternions are scaled using Slerp and the identity
        quaternions.
        """
        
        """ If Quaternions type do Quaternions * Quaternions """
        if isinstance(other, Quaternions):
            sqs, oqs = Quaternions._broadcast(self.qs, other.qs)

            q0 = sqs[...,0]; q1 = sqs[...,1]; 
            q2 = sqs[...,2]; q3 = sqs[...,3]; 
            r0 = oqs[...,0]; r1 = oqs[...,1]; 
            r2 = oqs[...,2]; r3 = oqs[...,3]; 
            
            qs = np.empty(sqs.shape)
            qs[...,0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3
            qs[...,1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2
            qs[...,2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1
            qs[...,3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0
            
            return Quaternions(qs)
        
        """ If array type do Quaternions * Vectors """
        if isinstance(other, np.ndarray) and other.shape[-1] == 3:
            vs = Quaternions(np.concatenate([np.zeros(other.shape[:-1] + (1,)), other], axis=-1))

            return (self * (vs * -self)).imaginaries

        """ If float do Quaternions * Scalars """
        if isinstance(other, np.ndarray) or isinstance(other, float):
            return Quaternions.slerp(Quaternions.id_like(self), self, other)
        
        raise TypeError('Cannot multiply/add Quaternions with type %s' % str(type(other)))
        
    def __div__(self, other):
        """
        When a Quaternion type is supplied, division is defined
        as multiplication by the inverse of that Quaternion.
        
        When a scalar or vector is supplied it is defined
        as multiplicaion of one over the supplied value.
        Essentially a scaling.
        """
        
        if isinstance(other, Quaternions): return self * (-other)
        if isinstance(other, np.ndarray): return self * (1.0 / other)
        if isinstance(other, float): return self * (1.0 / other)
        raise TypeError('Cannot divide/subtract Quaternions with type %s' + str(type(other)))
        
    def __eq__(self, other): return self.qs == other.qs
    def __ne__(self, other): return self.qs != other.qs
    
    def __neg__(self):
        """ Invert Quaternions """
        return Quaternions(self.qs * np.array([[1, -1, -1, -1]]))
    
    def __abs__(self):
        """ Unify Quaternions To Single Pole """
        qabs = self.normalized().copy()
        top = np.sum(( qabs.qs) * np.array([1,0,0,0]), axis=-1)
        bot = np.sum((-qabs.qs) * np.array([1,0,0,0]), axis=-1)
        qabs.qs[top < bot] = -qabs.qs[top <  bot]
        return qabs
    
    def __iter__(self): return iter(self.qs)
    def __len__(self): return len(self.qs)
    
    def __getitem__(self, k):    return Quaternions(self.qs[k]) 
    def __setitem__(self, k, v): self.qs[k] = v.qs
        
    @property
    def lengths(self):
        return np.sum(self.qs**2.0, axis=-1)**0.5
    
    @property
    def reals(self):
        return self.qs[...,0]
        
    @property
    def imaginaries(self):
        return self.qs[...,1:4]
    
    @property
    def shape(self): return self.qs.shape[:-1]
    
    def repeat(self, n, **kwargs):
        return Quaternions(self.qs.repeat(n, **kwargs))
    
    def normalized(self):
        return Quaternions(self.qs / self.lengths[...,np.newaxis])
    
    def log(self):
        norm = abs(self.normalized())
        imgs = norm.imaginaries
        lens = np.sqrt(np.sum(imgs**2, axis=-1))
        lens = np.arctan2(lens, norm.reals) / (lens + 1e-10)
        return imgs * lens[...,np.newaxis]
    
    def constrained(self, axis):
        
        rl = self.reals
        im = np.sum(axis * self.imaginaries, axis=-1)
        
        t1 = -2 * np.arctan2(rl, im) + np.pi
        t2 = -2 * np.arctan2(rl, im) - np.pi
        
        top = Quaternions.exp(axis[np.newaxis] * (t1[:,np.newaxis] / 2.0))
        bot = Quaternions.exp(axis[np.newaxis] * (t2[:,np.newaxis] / 2.0))
        img = self.dot(top) > self.dot(bot)
        
        ret = top.copy()
        ret[ img] = top[ img]
        ret[~img] = bot[~img]
        return ret
    
    def constrained_x(self): return self.constrained(np.array([1,0,0]))
    def constrained_y(self): return self.constrained(np.array([0,1,0]))
    def constrained_z(self): return self.constrained(np.array([0,0,1]))
    
    def dot(self, q): return np.sum(self.qs * q.qs, axis=-1)
    
    def copy(self): return Quaternions(np.copy(self.qs))
    
    def reshape(self, s):
        self.qs.reshape(s)
        return self
    
    def interpolate(self, ws):
        return Quaternions.exp(np.average(abs(self).log, axis=0, weights=ws))
    
    def euler(self, order='xyz'):
        
        q = self.normalized().qs
        q0 = q[...,0]
        q1 = q[...,1]
        q2 = q[...,2]
        q3 = q[...,3]
        es = np.zeros(self.shape + (3,))
        
        if   order == 'xyz':
            es[..., 2] = np.arctan2(2 * (q0 * q3 - q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
            es[..., 1] = np.arcsin((2 * (q1 * q3 + q0 * q2)).clip(-1,1))
            es[..., 0] = np.arctan2(2 * (q0 * q1 - q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
        else:
            raise NotImplementedError('Cannot convert from ordering %s' % order)

        
        # https://github.com/ehsan/ogre/blob/master/OgreMain/src/OgreMatrix3.cpp
        # Use this class and convert from matrix
        
        return es
        
    
    def average(self):
        
        if len(self.shape) == 1:
            
            import numpy.core.umath_tests as ut
            system = ut.matrix_multiply(self.qs[:,:,np.newaxis], self.qs[:,np.newaxis,:]).sum(axis=0)
            w, v = np.linalg.eigh(system)
            qiT_dot_qref = (self.qs[:,:,np.newaxis] * v[np.newaxis,:,:]).sum(axis=1)
            return Quaternions(v[:,np.argmin((1.-qiT_dot_qref**2).sum(axis=0))])            
        
        else:
            
            raise NotImplementedError('Cannot average multi-dimensionsal Quaternions')

    def angle_axis(self):
        
        norm = self.normalized()        
        s = np.sqrt(1 - (norm.reals**2.0))
        s[s == 0] = 0.001
        
        angles = 2.0 * np.arccos(norm.reals)
        axis = norm.imaginaries / s[...,np.newaxis]
        
        return angles, axis
        
    
    def transforms(self):
        
        qw = self.qs[...,0]
        qx = self.qs[...,1]
        qy = self.qs[...,2]
        qz = self.qs[...,3]
        
        x2 = qx + qx; y2 = qy + qy; z2 = qz + qz
        xx = qx * x2; yy = qy * y2; wx = qw * x2
        xy = qx * y2; yz = qy * z2; wy = qw * y2
        xz = qx * z2; zz = qz * z2; wz = qw * z2

        m = np.empty(self.shape + (3,3))
        m[...,0,0] = 1.0 - (yy + zz)
        m[...,0,1] = xy - wz
        m[...,0,2] = xz + wy
        m[...,1,0] = xy + wz
        m[...,1,1] = 1.0 - (xx + zz)
        m[...,1,2] = yz - wx
        m[...,2,0] = xz - wy
        m[...,2,1] = yz + wx
        m[...,2,2] = 1.0 - (xx + yy)
        
        return m
    
    def ravel(self):
        return self.qs.ravel()
    
    @classmethod
    def id(cls, n):
        
        if isinstance(n, tuple):
            qs = np.zeros(n + (4,))
            qs[...,0] = 1.0
            return Quaternions(qs)
        
        if isinstance(n, int) or isinstance(n, long):
            qs = np.zeros((n,4))
            qs[:,0] = 1.0
            return Quaternions(qs)
        
        raise TypeError('Cannot Construct Quaternion from %s type' % str(type(n)))

    @classmethod
    def id_like(cls, a):
        qs = np.zeros(a.shape + (4,))
        qs[...,0] = 1.0
        return Quaternions(qs)
        
    @classmethod
    def exp(cls, ws):
    
        ts = np.sum(ws**2.0, axis=-1)**0.5
        ts[ts == 0] = 0.001
        ls = np.sin(ts) / ts
        
        qs = np.empty(ws.shape[:-1] + (4,))
        qs[...,0] = np.cos(ts)
        qs[...,1] = ws[...,0] * ls
        qs[...,2] = ws[...,1] * ls
        qs[...,3] = ws[...,2] * ls
        
        return Quaternions(qs).normalized()
        
    @classmethod
    def slerp(cls, q0s, q1s, a):
        
        fst, snd = cls._broadcast(q0s.qs, q1s.qs)
        fst, a = cls._broadcast(fst, a, scalar=True)
        snd, a = cls._broadcast(snd, a, scalar=True)
        
        len = np.sum(fst * snd, axis=-1)
        
        neg = len < 0.0
        len[neg] = -len[neg]
        snd[neg] = -snd[neg]
        
        amount0 = np.zeros(a.shape)
        amount1 = np.zeros(a.shape)

        linear = (1.0 - len) < 0.01
        omegas = np.arccos(len[~linear])
        sinoms = np.sin(omegas)
        
        amount0[ linear] = 1.0 - a[linear]
        amount1[ linear] =       a[linear]
        amount0[~linear] = np.sin((1.0 - a[~linear]) * omegas) / sinoms
        amount1[~linear] = np.sin(       a[~linear]  * omegas) / sinoms
        
        return Quaternions(
            amount0[...,np.newaxis] * fst + 
            amount1[...,np.newaxis] * snd)
    
    @classmethod
    def between(cls, v0s, v1s):
        a = np.cross(v0s, v1s)
        w = np.sqrt((v0s**2).sum(axis=-1) * (v1s**2).sum(axis=-1)) + (v0s * v1s).sum(axis=-1)
        return Quaternions(np.concatenate([w[...,np.newaxis], a], axis=-1)).normalized()
    
    @classmethod
    def from_angle_axis(cls, angles, axis):
        axis    = axis / (np.sqrt(np.sum(axis**2, axis=-1)) + 1e-10)[...,np.newaxis]
        sines   = np.sin(angles / 2.0)[...,np.newaxis]
        cosines = np.cos(angles / 2.0)[...,np.newaxis]
        return Quaternions(np.concatenate([cosines, axis * sines], axis=-1))
    
    @classmethod
    def from_euler(cls, es, order='xyz', world=False):
    
        axis = {
            'x' : np.array([1,0,0]),
            'y' : np.array([0,1,0]),
            'z' : np.array([0,0,1]),
        }
        
        q0s = Quaternions.from_angle_axis(es[...,0], axis[order[0]])
        q1s = Quaternions.from_angle_axis(es[...,1], axis[order[1]])
        q2s = Quaternions.from_angle_axis(es[...,2], axis[order[2]])
        
        return (q2s * (q1s * q0s)) if world else (q0s * (q1s * q2s))
    
    @classmethod
    def from_transforms(cls, ts):
        
        d0, d1, d2 = ts[...,0,0], ts[...,1,1], ts[...,2,2]
        
        q0 = ( d0 + d1 + d2 + 1.0) / 4.0
        q1 = ( d0 - d1 - d2 + 1.0) / 4.0
        q2 = (-d0 + d1 - d2 + 1.0) / 4.0
        q3 = (-d0 - d1 + d2 + 1.0) / 4.0
        
        q0 = np.sqrt(q0.clip(0,None))
        q1 = np.sqrt(q1.clip(0,None))
        q2 = np.sqrt(q2.clip(0,None))
        q3 = np.sqrt(q3.clip(0,None))
        
        c0 = (q0 >= q1) & (q0 >= q2) & (q0 >= q3)
        c1 = (q1 >= q0) & (q1 >= q2) & (q1 >= q3)
        c2 = (q2 >= q0) & (q2 >= q1) & (q2 >= q3)
        c3 = (q3 >= q0) & (q3 >= q1) & (q3 >= q2)
        
        q1[c0] *= np.sign(ts[c0,2,1] - ts[c0,1,2])
        q2[c0] *= np.sign(ts[c0,0,2] - ts[c0,2,0])
        q3[c0] *= np.sign(ts[c0,1,0] - ts[c0,0,1])
        
        q0[c1] *= np.sign(ts[c1,2,1] - ts[c1,1,2])
        q2[c1] *= np.sign(ts[c1,1,0] + ts[c1,0,1])
        q3[c1] *= np.sign(ts[c1,0,2] + ts[c1,2,0])  
        
        q0[c2] *= np.sign(ts[c2,0,2] - ts[c2,2,0])
        q1[c2] *= np.sign(ts[c2,1,0] + ts[c2,0,1])
        q3[c2] *= np.sign(ts[c2,2,1] + ts[c2,1,2])  
        
        q0[c3] *= np.sign(ts[c3,1,0] - ts[c3,0,1])
        q1[c3] *= np.sign(ts[c3,2,0] + ts[c3,0,2])
        q2[c3] *= np.sign(ts[c3,2,1] + ts[c3,1,2])  
        
        qs = np.empty(ts.shape[:-2] + (4,))
        qs[...,0] = q0
        qs[...,1] = q1
        qs[...,2] = q2
        qs[...,3] = q3
        
        return cls(qs)
