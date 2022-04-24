from bvh_parser import BVH_file_Py
from bvh_writer import BVH_writer_Py
from libcpp.string cimport string
import torch


#BVH_file

cdef class BVH_file:
    cdef public int* topology
    cdef public string* jointNames
    cdef public string* eeNames
    cdef public float** offsets
    cdef public float*** positions
    cdef public float*** rotations
    cdef public int jointNum
    cdef public int frameNum
    cdef dict __dict__ 

    cdef public void init(string filePath, string* jointNames, string* eeNames):
        self.pyFile = BVH_file_Py(filePath, jointNames, eeNames)
        self.jointNum = self.pyFile.jointNum
        self.frameNum = len(self.pyFile.anim.rotations)

        #topology
        pyTopology = self.pyFile.topology()
        self.topology = new int[len(pyTopology)]
        for i in range(len(pyTopology)):
            self.topology[i] = pyTopology[i]

        #jointNames
        self.jointNames = new string[len(self.pyFile.names)]
        for i in range(len(self.pyFile.names)):
            self.jointNames[i] = self.pyFile.names[i]
        
        ##eeNames
        self.eeNames = new string[len(eeNames)]
        for i in range(len(eeNames)):
            self.eeNames[i] = eeNames[i]
        
        #offsets (Joints x 3)
        pyOffsets = self.pyFile.offsets()
        self.offsets = new float[len(pyOffsets)][3]
        for i in range(len(pyOffsets)):
            for j in range(3):
                self.offsets[i][j] = pyOffsets[i][j]
        
        #positions (Frames x Joints x 3)
        pyPositions = self.pyFile.get_positions()
        self.positions = new float[self.frameNum][self.jointNum][3]
        for i in range(self.frameNum):
            for j in range(self.jointNum):
                for k in range(3):
                    self.positions[i][j][k] = pyPositions[i][j][k]
        
        #rotations (Frames x Joints x 3)
        pyRotations = self.pyFile.anim.rotations[:, self.pyFile.joints, :]
        self.rotations = new float[self.frameNum][self.jointNum][3]
        for i in range(self.frameNum):
            for j in range(self.jointNum):
                for k in range(3):
                    self.rotations[i][j][k] = pyRotations[i][j][k]

#BVH_writer

cdef class BVH_writer:
    cdef public string stdbvhPath
    cdef dict __dict__ 

    cdef public void init(string stdbvhPath):
        self.pyWriter = BVH_writer_Py(stdbvhPath)


    #rotations -> F x J x 3, positions -> F x 3
    cdef public void write(object writer, float*** rotations, float** positions, path, float frametime):
        pyRotations = torch.zeros((len(rotations), len(rotations[0]), 3))
        pyPositions = torch.zeros((len(positions), 3))
        for i in range(len(rotations)):
            for j in range((len(rotations[0]))):
                for k in range(3):
                    pyRotations[i][j][k] = rotations[i][j][k]
        for i in range(len(positions)):
            for j in range(3):
                pyPositions[i][j] = positions[i][j]
        self.pyWriter.write(rotations=pyRotations, positions=pyPositions, path=path, frametime=1.0/30):