import numpy as np
import torch
from Quaternions import Quaternions
from source.CharacterNavigation.blender_rendering.load_bvh_pair import BVH_file


# rotation with shape frame * J * 3
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


class BVH_writer():
    def __init__(self, stdbvhPath):
        staticFile = BVH_file(stdbvhPath)
        self.parent = staticFile.topology
        self.offset = staticFile.offsets
        self.names = staticFile.names
        self.joint_num = len(self.parent)

    # position, rotation with shape T * J * (3/4)
    def write(self, rotations, positions, order, path, frametime=1.0/30): #offsets incluye la raiz
        if order == 'quaternion':
            norm = rotations[:, :, 0] ** 2 + rotations[:, :, 1] ** 2 + rotations[:, :, 2] ** 2 + rotations[:, :, 3] ** 2
            norm = np.repeat(norm[:, :, np.newaxis], 4, axis=2)
            rotations /= norm
            rotations = Quaternions(rotations)
            rotations = np.degrees(rotations.euler())
            order = 'xyz'
        
        if rotations.shape[1] < self.joint_num:
            rootRot = torch.zeros((rotations.shape[0], 1, 3))
            rotations = torch.cat((rootRot, rotations), 1)

        return write_bvh(self.parent, self.offset, rotations, positions, self.names, frametime, order, path)

    def write_raw(self, motion, order, path, frametime=1.0/30):
        motion = motion.permute(1, 0).detach().cpu().numpy()
        positions = motion[:, -3:]
        rotations = motion[:, :-3]
        if order == 'quaternion':
            rotations = rotations.reshape((motion.shape[0], -1, 4))
        else:
            rotations = rotations.reshape((motion.shape[0], -1, 3))

        return self.write(rotations, positions, order, path, frametime)
