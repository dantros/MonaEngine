#include "BVHManager.hpp"
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include "../Core/Log.hpp"

namespace Mona{

    // BVHManager

    void BVHManager::StartUp() {
        if (PyImport_AppendInittab("cython_interface", PyInit_cython_interface) != 0) {
            fprintf(stderr, "Unable to extend Python inittab");
        }
        Py_Initialize();   // initialize Python
        std::string file_path = __FILE__;
        std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
        std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
        std::string src_path = "'" + dir_path + std::string("/bvh_python/pySrc") + "'";
        std::string pyLine = std::string("sys.path.append(") + src_path + std::string(")");
        PyRun_SimpleString("import sys");
        PyRun_SimpleString(pyLine.data());

        if (PyImport_ImportModule("cython_interface") == NULL) {
            fprintf(stderr, "Unable to import cython module.\n");
            if (PyErr_Occurred()) {
                PyErr_PrintEx(0);
            }
            else {
                fprintf(stderr, "Unknown error");
            }
        }
    }
    void BVHManager::ShutDown() {
        //Al cerrar el motor se llama esta función donde se limpia el mapa de animaciones
        m_bvhDataMap.clear();
        Py_FinalizeEx();
    }

    void BVHManager::CleanUnusedBVHClips() noexcept {
        /*
        * Elimina todos los punteros del mapa cuyo conteo de referencias es igual a uno,
        * es decir, que el puntero del mapa es el unico que apunta a esa memoria.
        */
        for (auto i = m_bvhDataMap.begin(), last = m_bvhDataMap.end(); i != last;) {
            if (i->second.use_count() == 1) {
                i = m_bvhDataMap.erase(i);
            }
            else {
                ++i;
            }

        }
    }
    
    std::shared_ptr<BVHData> BVHManager::readBVH(std::string modelName, std::string animName) {
        std::shared_ptr<BVHData> savedPtr = getBVHData(modelName, animName);
        if (savedPtr != nullptr) {
            return savedPtr;
        }
        BVHData* dataPtr = new BVHData(modelName, animName);
        std::shared_ptr<BVHData> sharedPtr = std::shared_ptr<BVHData>(dataPtr);
        m_bvhDataMap.insert({ {modelName, animName}, sharedPtr });
        return sharedPtr;
    }
    std::shared_ptr<BVHData> BVHManager::readBVH(std::shared_ptr<AnimationClip> animation) {
        std::shared_ptr<BVHData> savedPtr = getBVHData(animation);
        if (savedPtr != nullptr) {
            return savedPtr;
        }
        BVHData* dataPtr = new BVHData(animation);
        std::shared_ptr<BVHData> sharedPtr = std::shared_ptr<BVHData>(dataPtr);
        m_bvhDataMap.insert({ {animation->GetSkeleton()->GetModelName(), animation->GetAnimationName()}, sharedPtr });
        return sharedPtr;
    }

    std::shared_ptr<BVHData> BVHManager::getBVHData(std::string modelName, std::string animName) {
        if (m_bvhDataMap.find({ modelName, animName }) == m_bvhDataMap.end()) { return nullptr; }
        else { return m_bvhDataMap.at({modelName, animName}); }
    }
    std::shared_ptr<BVHData> BVHManager::getBVHData(std::shared_ptr<AnimationClip> animation) {
        return getBVHData(animation->GetSkeleton()->GetModelName(), animation->GetAnimationName());
    }
    
    void BVHManager::writeBVHDynamicData(std::shared_ptr<BVHData> data, std::string outAnimName) {
        std::string writePath = BVHData::_getFilePath(data->getModelName(), outAnimName);
        
        BVH_writer_interface* pyWriterPtr = createWriterInterface(PyUnicode_FromString(data->getInputFilePath().data()));
        // rotations
        PyObject* frameListRot = PyList_New(data->getFrameNum());
        for (unsigned int i = 0; i < data->getFrameNum(); i++) {
            PyObject* jointListRot = PyList_New(data->getJointNum());
            PyList_SET_ITEM(frameListRot, i, jointListRot);
            for (unsigned int j = 0; j < data->getJointNum(); j++) {
                PyObject* valListRot = PyList_New(4);
                PyList_SET_ITEM(jointListRot, j, valListRot);

                PyObject* valRotW = PyFloat_FromDouble((double)data->getDynamicRotations()[i][j].w());
                PyObject* valRotX = PyFloat_FromDouble((double)data->getDynamicRotations()[i][j].x());
                PyObject* valRotY = PyFloat_FromDouble((double)data->getDynamicRotations()[i][j].y());
                PyObject* valRotZ = PyFloat_FromDouble((double)data->getDynamicRotations()[i][j].z());
                PyList_SET_ITEM(valListRot, 0, valRotW);
                PyList_SET_ITEM(valListRot, 1, valRotX);
                PyList_SET_ITEM(valListRot, 2, valRotY);
                PyList_SET_ITEM(valListRot, 3, valRotZ);
            }
        }


        // positions
        PyObject* frameListPos = PyList_New(data->getFrameNum());
        for (unsigned int i = 0; i < data->getFrameNum(); i++) {
            PyObject* rootPos = PyList_New(3);
            PyList_SET_ITEM(frameListPos, i, rootPos);
            for (unsigned int j = 0; j < 3; j++) {
                PyObject* valRootPos = PyFloat_FromDouble((double)data->getDynamicRootPositions()[i](j));
                PyList_SET_ITEM(rootPos, j, valRootPos);
            }
        }
        writeBVH_interface(pyWriterPtr, frameListRot, frameListPos, PyUnicode_FromString(writePath.data()), PyFloat_FromDouble((double)data->getDynamicFrametime()), PyBool_FromLong(1));
    }



    //BVHData
    BVHData::BVHData(std::string modelName, std::string animName) {
        std::string filePath = _getFilePath(modelName, animName);
        if (!std::filesystem::exists(filePath)) { 
            MONA_LOG_ERROR("BVHFile Error: Path does not exist -> {0}", filePath);
            return;
        }
        m_modelName = modelName;
        m_animName = animName;
        long jointSubset = 0;
        long quater = 1;
        BVH_file_interface* pyFilePtr = createFileInterface(PyUnicode_FromString(filePath.data()), PyBool_FromLong(jointSubset), PyBool_FromLong(quater));
        initFile(pyFilePtr);
        setDynamicData(m_rotations, m_rootPositions, m_frametime);
    }
    BVHData::BVHData(std::shared_ptr<AnimationClip> animation) : BVHData(animation->GetSkeleton()->GetModelName(), animation->GetAnimationName()) {
        m_baseClip = animation;
    }

    void BVHData::initFile(BVH_file_interface* pyFile) {
        m_jointNum = pyFile->jointNum;
        m_frameNum = pyFile->frameNum;
        m_frametime = pyFile->frametime;

        int jointNum = m_jointNum;
        int frameNum = m_frameNum;

        // topology
        m_topology = std::vector<int>(jointNum);
        if (PyList_Check(pyFile->topology)) {
            for (Py_ssize_t i = 0; i < jointNum; i++) {
                PyObject* value = PyList_GetItem(pyFile->topology, i);
                m_topology[i] = (int)PyLong_AsLong(value);
            }
        }
        else {
            MONA_LOG_ERROR("Passed PyObject pointer was not a list!");
        }

        // jointNames
        m_jointNames = std::vector<std::string>(jointNum);
        if (PyList_Check(pyFile->jointNames)) {
            for (Py_ssize_t i = 0; i < jointNum; i++) {
                PyObject* name = PyList_GetItem(pyFile->jointNames, i);
                m_jointNames[i]= PyUnicode_AsUTF8(name);
            }
        }
        else {
            MONA_LOG_ERROR("Passed PyObject pointer was not a list!");
        }
        
        

        // offsets
        m_offsets = std::vector<Vector3f>(jointNum);
        if (PyList_Check(pyFile->offsets)) {
            for (Py_ssize_t i = 0; i < jointNum; i++) {
                PyObject* jointOff = PyList_GetItem(pyFile->offsets, i);
                for (Py_ssize_t j = 0; j < 3; j++) {
                    PyObject* valOff = PyList_GetItem(jointOff, j);
                    m_offsets[i][j] = (float)PyFloat_AsDouble(valOff);
                }

            }
        }
        else {
            MONA_LOG_ERROR("Passed PyObject pointer was not a list!");
        }

        // rotations
        m_rotations = std::vector<std::vector<Quaternion>>(frameNum);
        if (PyList_Check(pyFile->rotations)) {
            for (Py_ssize_t i = 0; i <frameNum; i++) {
                PyObject* frameRot = PyList_GetItem(pyFile->rotations, i);
                m_rotations[i] = std::vector<Quaternion>(jointNum);
                for (Py_ssize_t j = 0; j < jointNum; j++) {
                    PyObject* jointRot = PyList_GetItem(frameRot, j);
                    PyObject* valRotW = PyList_GetItem(jointRot, 0);
                    PyObject* valRotX = PyList_GetItem(jointRot, 1);
                    PyObject* valRotY = PyList_GetItem(jointRot, 2);
                    PyObject* valRotZ = PyList_GetItem(jointRot, 3);
                    m_rotations[i][j].w() = (float)PyFloat_AsDouble(valRotW);
                    m_rotations[i][j].x() = (float)PyFloat_AsDouble(valRotX);
                    m_rotations[i][j].y() = (float)PyFloat_AsDouble(valRotY);
                    m_rotations[i][j].z() = (float)PyFloat_AsDouble(valRotZ);
                }
            }
        }
        else {
            MONA_LOG_ERROR("Passed PyObject pointer was not a list!");
        }


        // positions
        m_rootPositions = std::vector<Vector3f>(frameNum);
        if (PyList_Check(pyFile->rootPositions)) {
            for (Py_ssize_t i = 0; i < frameNum; i++) {
                PyObject* frameRoot = PyList_GetItem(pyFile->rootPositions, i);
                for (Py_ssize_t j = 0; j < 3; j++) {
                    PyObject* valRoot = PyList_GetItem(frameRoot, j);
                    m_rootPositions[i][j] = (float)PyFloat_AsDouble(valRoot);
                }

            }
        }
        else {
            MONA_LOG_ERROR("Passed PyObject pointer was not a list!");
        }
    }

    void BVHData::setDynamicData(std::vector<std::vector<Quaternion>> rotations, std::vector<Vector3f> rootPositions, float frametime) {
        if (rotations.size() != rootPositions.size() || rotations.size() != m_frameNum) {
            MONA_LOG_ERROR("Dynamic data must fit static data!");
        }
        for (int i = 0; i < m_frameNum; i++) {
            if (rotations[i].size() != m_jointNum) {
                MONA_LOG_ERROR("Dynamic data must fit static data!");
            }
        }
        m_rootPositions_dmic = rootPositions;
        m_rotations_dmic = rotations;
        m_frametime_dmic = frametime;
    }

    std::vector<JointPose> BVHData::getFramePoses(int frame) {
        std::vector<JointPose> poses(m_jointNum);
        for (int i = 0; i < m_jointNum; i++) {
            glm::vec3 tr(m_offsets[i][0], m_offsets[i][1], m_offsets[i][2]);
            glm::vec3 scl(1, 1, 1);
            glm::fquat rot(m_rotations_dmic[frame][i].w(), m_rotations_dmic[frame][i].x(), m_rotations_dmic[frame][i].y(), m_rotations_dmic[frame][i].z());
            poses[i] = JointPose(rot, tr, scl);
        }
        return poses;
    }

}

