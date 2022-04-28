# distutils: language = c++
# cython: language_level=3
from libc.stdio cimport printf
import numpy as np
from bvh_handler import BVH_file, BVH_writer


#BVH_file

cdef public class BVH_file_interface[object BVH_file_interface, type BVH_file_interface_type]:
    cdef public object topology
    cdef public object jointNames
    cdef public object offsets
    cdef public object positions
    cdef public object rotations
    cdef public int jointNum
    cdef public int frameNum
    cdef public float frametime

cdef public void initFileInterface(BVH_file_interface fileInterface, filePath, jointNames):
    printf("inter0")
    pyFile = BVH_file(filePath, jointNames)
    fileInterface.jointNum = pyFile.jointNum
    fileInterface.frameNum = len(pyFile.anim.rotations)
    fileInterface.frametime = pyFile.frametime
    printf("inter1")

    #topology (jointNum)
    fileInterface.topology = list(pyFile.topology())
    printf("inter2")


    #jointNames (jointNum)
    fileInterface.jointNames = list(pyFile.names)
    printf("inter3")

    #offsets (JointNum x 3)
    fileInterface.offsets = pyFile.offsets().tolist()
    printf("inter4")

    #positions (FrameNum x JointNum x 3)
    fileInterface.positions = pyFile.get_positions().tolist()
    printf("inter5")

    #rotations (FrameNum x JointNum x 3)
    fileInterface.rotations = pyFile.anim.rotations[:, pyFile.joints, :].tolist()
    printf("inter6")


#BVH_writer

cdef public class BVH_writer_interface[object BVH_writer_interface, type BVH_writer_interface_type]:
    cdef public int jointNum
    cdef public staticDataPath

cdef public void initWriterInterface(BVH_writer_interface writerInterface, staticDataPath):
        pyFile = BVH_writer(staticDataPath)
        writerInterface.jointNum = pyFile.joint_num
        writerInterface.staticDataPath = staticDataPath

#rotations -> F x J x 3, positions -> F x 3
cdef public void writeBVH_interface(BVH_writer_interface writerInterface, rotations, positions, writePath, frametime):
    tRotations = np.array(rotations)
    tPositions = np.array(positions)
    pyWriter = BVH_writer(writerInterface.staticDataPath)
    pyWriter.write(rotations=tRotations, positions=tPositions, path=writePath, frametime=frametime)