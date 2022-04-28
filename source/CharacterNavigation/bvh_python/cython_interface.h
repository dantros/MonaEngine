/* Generated by Cython 0.29.28 */

#ifndef __PYX_HAVE__cython_interface
#define __PYX_HAVE__cython_interface

#include "Python.h"
struct BVH_file_interface;
struct BVH_writer_interface;

/* "cython_interface.pyx":11
 * #BVH_file
 * 
 * cdef public class BVH_file_interface[object BVH_file_interface, type BVH_file_interface_type]:             # <<<<<<<<<<<<<<
 *     cdef public object topology
 *     cdef public object jointNames
 */
struct BVH_file_interface {
  PyObject_HEAD
  PyObject *topology;
  PyObject *jointNames;
  PyObject *offsets;
  PyObject *positions;
  PyObject *rotations;
  int jointNum;
  int frameNum;
  float frametime;
};

/* "cython_interface.pyx":55
 * #BVH_writer
 * 
 * cdef public class BVH_writer_interface[object BVH_writer_interface, type BVH_writer_interface_type]:             # <<<<<<<<<<<<<<
 *     cdef public int jointNum
 *     cdef public staticDataPath
 */
struct BVH_writer_interface {
  PyObject_HEAD
  int jointNum;
  PyObject *staticDataPath;
};

#ifndef __PYX_HAVE_API__cython_interface

#ifndef __PYX_EXTERN_C
  #ifdef __cplusplus
    #define __PYX_EXTERN_C extern "C"
  #else
    #define __PYX_EXTERN_C extern
  #endif
#endif

#ifndef DL_IMPORT
  #define DL_IMPORT(_T) _T
#endif

__PYX_EXTERN_C DL_IMPORT(PyTypeObject) BVH_file_interface_type;
__PYX_EXTERN_C DL_IMPORT(PyTypeObject) BVH_writer_interface_type;

__PYX_EXTERN_C void initFileInterface(struct BVH_file_interface *, PyObject *, PyObject *);
__PYX_EXTERN_C void initWriterInterface(struct BVH_writer_interface *, PyObject *);
__PYX_EXTERN_C void writeBVH_interface(struct BVH_writer_interface *, PyObject *, PyObject *, PyObject *, PyObject *);

#endif /* !__PYX_HAVE_API__cython_interface */

/* WARNING: the interface of the module init function changed in CPython 3.5. */
/* It now returns a PyModuleDef instance instead of a PyModule instance. */

#if PY_MAJOR_VERSION < 3
PyMODINIT_FUNC initcython_interface(void);
#else
PyMODINIT_FUNC PyInit_cython_interface(void);
#endif

#endif /* !__PYX_HAVE__cython_interface */
