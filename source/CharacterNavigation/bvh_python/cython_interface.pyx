# distutils: language = c++
# cython: language_level=3
from bvh_parser import BVH_file
from bvh_writer import BVH_writer
from torch import tensor


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

cdef public object createFileObject():
    return BVH_file_interface()

cdef public void initFileInterface(BVH_file_interface fileInterface, filePath, jointNames):
    pyFile = BVH_file(filePath, jointNames)
    fileInterface.jointNum = pyFile.jointNum
    fileInterface.frameNum = len(pyFile.anim.rotations)
    fileInterface.frametime = pyFile.frametime

    #topology (jointNum)
    fileInterface.topology = list(pyFile.topology())

    #jointNames (jointNum)
    fileInterface.jointNames = list(pyFile.names)
    
    #offsets (JointNum x 3)
    fileInterface.offsets = pyFile.offsets().tolist()
    
    #positions (FrameNum x JointNum x 3)
    fileInterface.positions = pyFile.get_positions().tolist()
    
    #rotations (FrameNum x JointNum x 3)
    fileInterface.rotations = pyFile.anim.rotations[:, pyFile.joints, :].tolist()


#BVH_writer

cdef public class BVH_writer_interface[object BVH_writer_interface, type BVH_writer_interface_type]:
    cdef public int jointNum
    cdef public staticDataPath

cdef public object createWriterObject():
    return BVH_writer_interface()

cdef public void initWriterInterface(BVH_writer_interface writerInterface, staticDataPath):
        pyFile = BVH_writer(staticDataPath)
        writerInterface.jointNum = pyFile.joint_num
        writerInterface.staticDataPath = staticDataPath

#rotations -> F x J x 3, positions -> F x 3
cdef public void writeBVH_interface(BVH_writer_interface writerInterface, rotations, positions, writePath, frametime):
    tRotations = tensor(rotations)
    tPositions = tensor(positions)
    pyWriter = BVH_writer(writerInterface.staticDataPath)
    pyWriter.write(rotations=tRotations, positions=tPositions, path=writePath, frametime=frametime)