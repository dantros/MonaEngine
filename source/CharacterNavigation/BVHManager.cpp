#include "BVHManager.hpp"
#include <Python.h>
#include <stdexcept>
#include "PyUtils.h"
#include "cython_interface.h"

namespace Mona{

    BVH_file::BVH_file(std::string &filePath, std::vector<std::string> jointNames){
        Py_Initialize();   // initialize Python
        PyInit_cython_interface();
        BVH_file_interface pyFile = BVH_file_interface(PyUnicode_FromString( filePath.data()), NULL);
        if (jointNames != NULL){
            PyObject* listObj = PyList_New( jointNames.size() );
            if (!listObj) throw logic_error("Unable to allocate memory for Python list");
            for (unsigned int i = 0; i < jointNames.size(); i++) {
                PyObject *name = PyUnicode_FromString( jointNames[i].data());
                if (!name) {
                    Py_DECREF(listObj);
                    throw logic_error("Unable to allocate memory for Python list");
                }
                PyList_SET_ITEM(listObj, i, name);
            }
            pyFile = BVH_file_interface(PyUnicode_FromString( filePath.data()), listObj);
        }
        m_jointNum = pyFile.jointNum;
        m_frameNum = pyFile.frameNum;
        m_frametime = pyFile.frametime;
        
        // topology
        m_topology = new int[m_jointNum];
        if (PyList_Check(pyFile.topology)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.topology); i++) {
				PyObject *value = PyList_GetItem(pyFile.topology, i);
				m_topology[i] = PyInt_AsInt(value);
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}

        // jointNames
        m_jointNames = new std::string[m_jointNum];
        for(int i = 0; i < m_jointNum; i++){
            m_jointNames[i] = jointNames[i]
        }

        // offsets
        m_offsets = new float[m_jointNum][3];
        if (PyList_Check(pyFile.offsets)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.offsets); i++) {
				PyObject *jointOff = PyList_GetItem(pyFile.offsets, i);
                for(Py_ssize_t j = 0; j < PyList_Size(jointOff); j++) {
                    PyObject *valOff = PyList_GetItem(jointOff, j);
                    m_offsets[i][j] = PyFloat_AsFloat(valOff);
                }
				
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list!");
		}

        // rotations
        m_rotations = new float[m_frameNum][m_jointNum][3];
        if (PyList_Check(pyFile.rotations)) {
			for(Py_ssize_t i = 0; i < PyList_Size(pyFile.rotations); i++) {
				PyObject *frameRot = PyList_GetItem(pyFile.rotations, i);
                for(Py_ssize_t j = 0; j < PyList_Size(frameRot); j++) {
                    PyObject *jointRot = PyList_GetItem(frameRot, j);
                    for(Py_ssize_t k = 0; k < PyList_Size(jointRot); k++) {
                        PyObject *valRot = PyList_GetItem(jointRot, k);
                        m_rotations[i][j][k] = PyFloat_AsFloat(valRot);
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
				PyObject *framePos = PyList_GetItem(pyFile.positions, i);
                for(Py_ssize_t j = 0; j < PyList_Size(framePos); j++) {
                    PyObject *jointPos = PyList_GetItem(framePos, j);
                    for(Py_ssize_t k = 0; k < PyList_Size(jointPos); k++) {
                        PyObject *valPos = PyList_GetItem(jointPos, k);
                        m_positions[i][j][k] = PyFloat_AsFloat(valPos);
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

    void BVH_writer::write(float*** rotations, float** positions, float frametime, int frameNum, std::string& writePath){
        Py_Initialize();   // initialize Python
        PyInit_cython_interface();
        BVH_writer_interface pyWriter = BVH_writer_interface(PyUnicode_FromString( writePath.data()));
        int jointNum = pyWriter.jointNum;
        // rotations
        PyObject* frameListRot = PyList_New( frameNum.size() );
        for (unsigned int i = 0; i < frameNum; i++) {
            PyObject* jointListRot = PyList_New( jointNum );
            PyList_SET_ITEM(frameListRot, i, jointListRot);
            for (unsigned int j = 0; j < jointNum; j++) {
                PyObject* valLisRot = PyList_New( 3 );
                PyList_SET_ITEM(jointListRot, j, valListRot);
                for (unsigned int k = 0; k < 3; k++) {
                    PyObject* valRot = PyFloat_FromFloat(rotations[i][j][k]);
                    PyList_SET_ITEM(valListRot, k, valRot);
                }
            }
        }


        // positions
        PyObject* frameListPos = PyList_New( frameNum.size() );
        for (unsigned int i = 0; i < frameNum; i++) {
            PyObject* jointListPos = PyList_New( jointNum );
            PyList_SET_ITEM(frameListPos, i, jointListPos);
            for (unsigned int j = 0; j < jointNum; j++) {
                PyObject* valListPos = PyList_New( 3 );
                PyList_SET_ITEM(jointListPos, j, valListPos);
                for (unsigned int k = 0; k < 3; k++) {
                    PyObject* valPos = PyFloat_FromFloat(rotations[i][j][k]);
                    PyList_SET_ITEM(valListPos, k, valPos);
                }
            }
        }

        writeBVH_interface(frameListRot, frameListPos, PyUnicode_FromString(writePath), frametime)
        Py_Finalize();
    }

}

