#include <Python.h>

int
main(int argc, char *argv[])
{
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
    int i;
    Py_SetProgramName(argv[0]);
    Py_Initialize();
    PySys_SetArgv(argc, argv);

    PyRun_SimpleString("import os, sys\n"
                     "print sys.argv, \"\\n\".join(sys.path)\n"
                     "print os.getcwd()\n"
                     "import multiply\n");

    pName = PyString_FromString("multiply");
    /* Error checking of pName left out */

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "multiply");
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(2);

            pValue = PyInt_FromLong(atoi("5"));
            if (!pValue) {
              Py_DECREF(pArgs);
              Py_DECREF(pModule);
              fprintf(stderr, "Cannot convert argument\n");
              return 1;
            }
            /* pValue reference stolen here: */
            PyTuple_SetItem(pArgs, 0, pValue);
            
            pValue = PyInt_FromLong(atoi("4"));
            if (!pValue) {
              Py_DECREF(pArgs);
              Py_DECREF(pModule);
              fprintf(stderr, "Cannot convert argument\n");
              return 1;
            }
            /* pValue reference stolen here: */
            PyTuple_SetItem(pArgs, 1, pValue);

            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", "multiply");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "multiply");
        return 1;
    }
    Py_Finalize();
    return 0;
}