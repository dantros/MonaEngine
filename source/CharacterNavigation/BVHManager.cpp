#include "BVHManager.hpp"
#include <Python.h>
#include <vector>
#include <stdexcept>
#include "PyUtils.h"
#include "cython_interface.h"

namespace Mona{

    BVH_file::BVH_file(std::string &filePath, std::string* jointNames, std::string* eeNames){
        Py_Initialize();   // initialize Python
        PyInit_cython_interface();
        BVH_file_interface pyFile = BVH_file_interface();
        m_jointNum = pyFile.jointNum;
        m_frameNum = pyFile.frameNum;
        m_eeNum = pyFile.eeNum;
        m_frametime = pyFile.frametime;
        m_topology = new int[m_jointNum];
        m_jointNames = new std::string[m_jointNum];
        m_eeNames = new std::string[m_eeNum];
        m_offsets = new float[m_jointNum][3];
        m_positions = new float[m_frameNum][m_jointNum][3];
        m_rotations = new float[m_frameNum][m_jointNum][3];
        Py_Finalize();
    }

    BVH_writer::BVH_writer(std::string &staticDataPath){

    }

    void BVH_writer::write(float*** rotations, float** positions, std::string& writePath, float frametime){

    }

}

