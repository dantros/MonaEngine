#include "BVHManager.hpp"
#include <Python.h>
#include <vector>
#include <stdexcept>
#include "PyUtils.h"
#include "cython_interface.h"

namespace Mona{

    BVH_file::BVH_file(std::string &filePath, std::string* jointNames, std::string* eeNames){
        Py_Initialize();   // initialize Python
        PyInit_cython_interface();    // initialize module (initpycls(); in Py2)
        PyObject *obj = BVH_file_interface();
        for(int i=0; i<10; i++){
            addData(obj, i);
        }
        printf("%s\n", printCls(obj));
        Py_Finalize();
    }

    BVH_writer::BVH_writer(std::string &staticDataPath){

    }

    void BVH_writer::write(float*** rotations, float** positions, std::string& writePath, float frametime){

    }

}

