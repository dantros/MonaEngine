import os
import sys
sys.path.append(os.getcwd())
sys.path.append('../')
print(sys.path)

from utils import BVH
import numpy as np
import bpy
import mathutils
import math
import pdb

#scale factor for bone length
global_scale = 10

class BVH_file:
    def __init__(self, file_path):
        self.anim, self.names, self.frametime = BVH.load(file_path)

        #permute (x, y, z) to (z, x, y)
        tmp = self.anim.offsets.copy()
        self.anim.offsets[..., 0] = tmp[..., 2]
        self.anim.offsets[..., 1] = tmp[..., 0]
        self.anim.offsets[..., 2] = tmp[..., 1]

        tmp = self.anim.positions.copy()
        self.anim.positions[..., 0] = tmp[..., 2]
        self.anim.positions[..., 1] = tmp[..., 0]
        self.anim.positions[..., 2] = tmp[..., 1]

        tmp = self.anim.rotations.qs.copy()
        self.anim.rotations.qs[..., 1] = tmp[..., 3]
        self.anim.rotations.qs[..., 2] = tmp[..., 1]
        self.anim.rotations.qs[..., 3] = tmp[..., 2]

        self.joint_num = self.anim.rotations.shape[1]
        self.frame_num = self.anim.rotations.shape[0]

        self.normalize()

    @property
    def topology(self):
        return self.anim.parents

    @property
    def offsets(self):
        return self.anim.offsets

    # Normalize bone length by height and translate the (x, y) mean to (0, 0)
    def normalize(self):
        height = self.get_height() / global_scale
        self.anim.offsets /= height
        self.anim.positions /= height
        mean_position = np.mean(self.anim.positions[:, 0, :], axis=0)
        self.anim.positions[:, 0, 0] -= mean_position[0]
        self.anim.positions[:, 0, 1] -= mean_position[1]


    def get_height(self):
        low = high = 0

        def dfs(i, pos):
            nonlocal low
            nonlocal high
            low = min(low, pos[-1])
            high = max(high, pos[-1])

            for j in range(self.joint_num):
                if self.topology[j] == i:
                    dfs(j, pos + self.offsets[j])

        dfs(0, np.array([0, 0, 0]))

        return high - low


def add_bone(offset, parent_obj, name):
    center = parent_obj.location + offset / 2
    length = offset.dot(offset) ** 0.5

    base = mathutils.Vector((0., 0., 1.))
    target = offset.normalized()
    axis = base.cross(target)
    theta = np.math.acos(base.dot(target))
    rot = mathutils.Quaternion(axis, theta)

    bpy.ops.mesh.primitive_cone_add(vertices=5, radius1=0.022 * global_scale, radius2=0.0132 * global_scale, depth=length, enter_editmode=False, location=center)
    new_bone = bpy.context.object
    new_bone.name = name
    new_bone.rotation_mode = 'QUATERNION'
    new_bone.rotation_quaternion = rot

    set_parent(parent_obj, new_bone)

    return new_bone

def add_joint(location, parent_obj, name):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=3, radius=0.001, enter_editmode=False, location=location)
    new_joint = bpy.context.object
    if parent_obj is not None:
        set_parent(parent_obj, new_joint)
        pass
    new_joint.name = name

    return new_joint

def build_t_pose(file: BVH_file, joint, parent_obj, all_obj, rootOffset):
    if joint != 0:
        offset = mathutils.Vector(file.offsets[joint])
        new_bone = add_bone(offset, parent_obj, file.names[joint] + '_bone')
        new_joint = add_joint(parent_obj.location + offset, new_bone, file.names[joint] + '_end')
        all_obj.append(new_bone)
        all_obj.append(new_joint)
    else:
        new_joint = add_joint(rootOffset, None, file.names[joint] + '_end')
        all_obj.append(new_joint)

    for i in range(len(file.topology)):
        if file.topology[i] == joint:
            build_t_pose(file, i, new_joint, all_obj, rootOffset)


def set_parent(parent, child):
    child.parent = parent
    child.matrix_parent_inverse = parent.matrix_world.inverted()
    '''
        See https://blender.stackexchange.com/questions/9200/how-to-make-object-a-a-parent-of-object-b-via-blenders-python-api
    '''


def set_animation(inputFile, resultFile, joints, resultOffset):
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = inputFile.anim.rotations.shape[0] - 1

    bpy.context.scene.render.fps = 1 / inputFile.frametime

    bpy.ops.object.select_all(action='DESELECT')

    print('Set fps to', bpy.context.scene.render.fps)
    print(inputFile.frame_num, 'frames in total')
    resFactor = int(round((inputFile.frame_num / resultFile.frame_num), 0))
    print('factor: ' + str(resFactor))
    for frame in range(0, inputFile.frame_num):
        joints[0].location = inputFile.anim.positions[frame, 0, :]
        joints[0].keyframe_insert(data_path='location', frame=frame)
        if frame % 100 == 99:
            print('[{}/{}] done.'.format(frame+1, inputFile.frame_num))
        for j in range(inputFile.joint_num):
            joints[j].rotation_mode = 'QUATERNION'
            joints[j].rotation_quaternion = mathutils.Quaternion(inputFile.anim.rotations.qs[frame, j, :])
            joints[j].keyframe_insert(data_path='rotation_quaternion', frame=frame)
        
        if frame >= resultFile.frame_num:
            continue
        frame_amp = frame*resFactor
        for f in range(resFactor):   
            joints[inputFile.joint_num].location = resultFile.anim.positions[frame, 0, :] + resultOffset
            joints[inputFile.joint_num].keyframe_insert(data_path='location', frame=frame_amp+f)
        if frame % 100 == 99:
            print('[{}/{}] done.'.format(frame+1, inputFile.frame_num))
        for j in range(resultFile.joint_num):
            for f in range(resFactor):   
                joints[inputFile.joint_num + j].rotation_mode = 'QUATERNION'
                joints[inputFile.joint_num + j].rotation_quaternion = mathutils.Quaternion(resultFile.anim.rotations.qs[frame, j, :])
                joints[inputFile.joint_num + j].keyframe_insert(data_path='rotation_quaternion', frame=frame_amp+f)

    bpy.context.scene.frame_current = 0


def load_bvh(input, result):
    print('Loading BVH file......')
    inputFile = BVH_file(input)
    resultFile = BVH_file(result)
    print('Loading BVH file done.')

    print('Building T-Pose......')

    for n, name in enumerate(inputFile.names):
        inputFile.names[n] = 'input_' + name

    for n, name in enumerate(resultFile.names):
        resultFile.names[n] = 'result_' + name
    # input
    all_obj1 = []
    rootOffset = mathutils.Vector((0., 0., 0.))
    build_t_pose(inputFile, 0, None, all_obj1, rootOffset)

    # result
    all_obj2 = []
    rootOffset = mathutils.Vector((0., 15., 0.))
    build_t_pose(resultFile, 0, None, all_obj2, rootOffset)

    print('Building T-Pose done.')

    print('Loading keyframes......')

    #pairing object order and file.animation's order
    all_obj = all_obj1 + all_obj2 #concat
    all_joints = []
    for j in range(inputFile.joint_num):
        name = inputFile.names[j]
        for obj in all_obj1:
            if obj.name == name + '_end':
                all_joints.append(obj)
                break
    for j in range(resultFile.joint_num):
        name = resultFile.names[j]
        for obj in all_obj2:
            if obj.name == name + '_end':
                all_joints.append(obj)
                break
    
    resultOffset = [0, 15, 0]
    set_animation(inputFile, resultFile, all_joints, resultOffset)

    mat = bpy.data.materials.new("PKHG")
    mat.diffuse_color = (float(.5),1.0,0.0, 1.0)
    for obj in all_obj2:
        obj.active_material = mat
    
    print('Loading keyframes done.')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in all_obj:
        obj.select_set(True)
    bpy.ops.object.move_to_collection(collection_index=0, is_new=True, new_collection_name="Character")
    bpy.ops.object.select_all(action='DESELECT')
    print('Load bvh all done!')

    return all_obj


if __name__ == '__main__':
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    input = [f for f in os.listdir('./') if 'input.bvh' in f][0]
    result = [f for f in os.listdir('./') if 'result.bvh' in f][0]

    load_bvh(input, result)
