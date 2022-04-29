# distutils: language = c++
# cython: language_level=3
import numpy as np
from bvh_handler import BVH_file, BVH_writer

#BVH_file
cdef public class BVH_file_interface[object BVH_file_interface, type BVH_file_interface_type]:
    cdef public object topology
    cdef public object jointNames
    cdef public object offsets
    cdef public object rootPositions
    cdef public object rotations
    cdef public int jointNum
    cdef public int frameNum
    cdef public float frametime

cdef public BVH_file_interface createFileInterface(filePath, jointNames):
    fileInterface = BVH_file_interface()
    pyFile = BVH_file(filePath, jointNames)
    fileInterface.jointNum = pyFile.joint_num
    fileInterface.frameNum = len(pyFile.anim.rotations)
    fileInterface.frametime = pyFile.frametime

    #topology (jointNum)
    fileInterface.topology = list(pyFile.topology)

    #jointNames (jointNum)
    fileInterface.jointNames = list(pyFile.names)

    #offsets (JointNum x 3)
    fileInterface.offsets = pyFile.offsets.tolist()

    #positions (FrameNum x JointNum x 3)
    fileInterface.rootPositions = pyFile.get_root_positions().tolist()

    #rotations (FrameNum x JointNum x 3)
    fileInterface.rotations = pyFile.anim.rotations[:, pyFile.joints, :].tolist()

    return fileInterface


#BVH_writer

cdef public class BVH_writer_interface[object BVH_writer_interface, type BVH_writer_interface_type]:
    cdef public int jointNum
    cdef public staticDataPath

cdef public BVH_writer_interface createWriterInterface(staticDataPath):
        writerInterface = BVH_writer_interface()
        pyFile = BVH_writer(staticDataPath)
        writerInterface.jointNum = pyFile.joint_num
        writerInterface.staticDataPath = staticDataPath
        return writerInterface

#rotations -> F x J x 3, positions -> F x 3
cdef public void writeBVH_interface(BVH_writer_interface writerInterface, rotations, positions, writePath, frametime):
    tRotations = np.array(rotations)
    tPositions = np.array(positions)
    pyWriter = BVH_writer(writerInterface.staticDataPath)
    pyWriter.write(rotations=tRotations, positions=tPositions, path=writePath, frametime=frametime)