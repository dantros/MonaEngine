# distutils: language = c++
# cython: language_level=3
from bvh_parser import BVH_file
from bvh_writer import BVH_writer
from torch import tensor


#BVH_file

cdef public class BVH_file_interface[object BVH_file_interface, type BVH_file_interface_type]:
    cdef public object topology
    cdef public object jointNames
    cdef public object eeNames
    cdef public object offsets
    cdef public object positions
    cdef public object rotations
    cdef public int jointNum
    cdef public int frameNum
    cdef public int eeNum
    cdef public float frametime
    cdef dict __dict__
    pyFile = None

    cdef public void init(self, filePath, jointNames):
        pyFile = BVH_file(filePath, jointNames)
        self.jointNum = pyFile.jointNum
        self.frameNum = len(pyFile.anim.rotations)
        self.frametime = pyFile.frametime

        #topology (jointNum)
        self.topology = list(pyFile.topology())

        #jointNames (jointNum)
        self.jointNames = list(pyFile.names)
        
        #offsets (JointNum x 3)
        self.offsets = pyFile.offsets().tolist()
        
        #positions (FrameNum x JointNum x 3)
        self.positions = pyFile.get_positions().tolist()
        
        #rotations (FrameNum x JointNum x 3)
        self.rotations = pyFile.anim.rotations[:, pyFile.joints, :].tolist()


#BVH_writer

cdef public class BVH_writer_interface[object BVH_writer_interface, type BVH_writer_interface_type]:
    cdef public int jointNum
    cdef dict __dict__
    cdef public void init(self, staticDataPath):
            pyFile = BVH_writer(staticDataPath)
            self.jointNum = pyFile.joint_num
            self.staticDataPath = staticDataPath

    #rotations -> F x J x 3, positions -> F x 3
    cdef public void writeBVH_interface(self, rotations, positions, writePath, frametime):
        tRotations = tensor(rotations)
        tPositions = tensor(positions)
        pyWriter = BVH_writer(self.staticDataPath)
        pyWriter.write(rotations=tRotations, positions=tPositions, path=writePath, frametime=1.0/30)