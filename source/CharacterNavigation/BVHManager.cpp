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
        
        // topology
        m_topology = new int[m_jointNum];
        if (PyList_Check(pyFile.topology)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.topology); i++) {
				PyObject *value = PyList_GetItem(pyFile.topology, i);
				m_topology.push_back( PyFloat_AsInt(value) );
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}

        // jointNames
        m_jointNames = new std::string[m_jointNum];
        for(int i = 0; i < m_jointNum; i++){
            m_jointNames[i] = jointNames[i]
        }

        // eeNames
        m_eeNames = new std::string[m_eeNum];
        for(int i = 0; i < m_eeNum; i++){
            m_eeNames[i] = eeNames[i]
        }

        // offsets
        m_offsets = new float[m_jointNum][3];
        if (PyList_Check(pyFile.offsets)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.offsets); i++) {
				PyObject *joint = PyList_GetItem(pyFile.offsets, i);
                for(Py_ssize_t j = 0; j < PyList_Size(joint); j++) {
                    PyObject *val = PyList_GetItem(joint, j);
                    m_offsets[i][j] = PyFloat_AsFloat(val);
                }
				
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}

        // rotations
        m_rotations = new float[m_frameNum][m_jointNum][3];
        if (PyList_Check(pyFile.rotations)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.rotations); i++) {
				PyObject *frame = PyList_GetItem(pyFile.rotations, i);
                for(Py_ssize_t j = 0; j < PyList_Size(frame); j++) {
                    PyObject *joint = PyList_GetItem(frame, j);
                    for(Py_ssize_t k = 0; k < PyList_Size(joint); k++) {
                        PyObject *val = PyList_GetItem(joint, k);
                        m_rotations[i][j][k] = PyFloat_AsFloat(val);
                    }
                }
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}


        // positions
        m_positions = new float[m_frameNum][m_jointNum][3];
		if (PyList_Check(pyFile.positions)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.positions); i++) {
				PyObject *frame = PyList_GetItem(pyFile.positions, i);
                for(Py_ssize_t j = 0; j < PyList_Size(frame); j++) {
                    PyObject *joint = PyList_GetItem(frame, j);
                    for(Py_ssize_t k = 0; k < PyList_Size(joint); k++) {
                        PyObject *val = PyList_GetItem(joint, k);
                        m_positions[i][j][k] = PyFloat_AsFloat(val);
                    }
                }
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}
        Py_Finalize();
    }

    BVH_writer::BVH_writer(std::string &staticDataPath){
        m_staticDataPath = staticDataPath;
    }

    void BVH_writer::write(float*** rotations, float** positions, float frametime, std::string& writePath){
        Py_Initialize();   // initialize Python
        PyInit_cython_interface();
        writeBVH_interface()
        Py_Finalize();
    }

}

