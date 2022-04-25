# distutils: language = c++
from bvh_parser import BVH_file
from bvh_writer import BVH_writer
from torch import tensor


#BVH_file

cdef class BVH_file_interface:
    cdef public object topology
    cdef public object jointNames
    cdef public object eeNames
    cdef public object offsets
    cdef public object positions
    cdef public object rotations
    cdef public int jointNum
    cdef public int frameNum
    cdef public int eeNum
    cdef dict __dict__
    pyFile = None

    cdef public void init(self, filePath, jointNames, eeNames):
        self.pyFile = BVH_file(filePath, jointNames, eeNames)
        self.jointNum = self.pyFile.jointNum
        self.frameNum = len(self.pyFile.anim.rotations)

        #topology (jointNum)
        self.topology = self.pyFile.topology()

        #jointNames (jointNum)
        self.jointNames = self.pyFile.names
        
        ##eeNames (eeNum)
        self.eeNames = eeNames
        self.eeNum = len(eeNames)
        
        #offsets (JointNum x 3)
        self.offsets = self.pyFile.offsets()
        
        #positions (FrameNum x JointNum x 3)
        self.positions = self.pyFile.get_positions()
        
        #rotations (FrameNum x JointNum x 3)
        self.rotations = self.pyFile.anim.rotations[:, self.pyFile.joints, :]


#BVH_writer

#rotations -> F x J x 3, positions -> F x 3
cdef public void writeBVH_interface(stdbvhPath, rotations, positions, writePath, frametime):
    tRotations = tensor(rotations)
    tPositions = tensor(positions)
    pyWriter = BVH_writer(stdbvhPath)
    pyWriter.write(rotations=tRotations, positions=tPositions, path=writePath, frametime=1.0/30)