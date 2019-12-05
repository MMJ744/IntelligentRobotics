#include <pluginlib/class_list_macros.h>
 #include "global_planner.h"
 #include "nav_msgs/Path.h"
 #include <iostream>
 #include <Python.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace global_planner {

/*#include "globalPlanner.py.xxd"*/

 GlobalPlanner::GlobalPlanner (){

 }

 GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

 }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
   PyObject *pName, *pModule, *pFunc;
   PyObject *pArgs, *pValue;
   int i;
   string s = "globalplanner";

   int n = s.length(); 
   char char_array[n + 1]; 
   strcpy(char_array, s.c_str()); 
   Py_SetProgramName(char_array);
   Py_Initialize();
   PySys_SetArgv(0, nullptr);
   PyRun_SimpleString("import os, sys\n"
                     "print sys.argv, \"\\n\".join(sys.path)\n"
                     "print os.getcwd()\n"
                     "import globalPlanner\n");

   
   /*PyRun_SimpleString((const char *) globalPlanner_py);*/

   pName = PyString_FromString("globalplanner");
    /* Error checking of pName left out */

   pModule = PyImport_Import(pName);
   Py_DECREF(pName);
   if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "main");
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                PyObject *np_ret = reinterpret_cast<PyObject*>(pValue);

                
                // Convert back to C++ array and print.
                int len = PyList_Size(np_ret);
                /*c_out = reinterpret_cast<long double*>(PyArray_DATA(np_ret));
                std::cout << "Printing output array" << std::endl;
                for (int i = 0; i < len; i++)
                    std::cout << c_out[i] << ' ';
                */

                if (!PyList_Check(np_ret)) {
                    std::cout << "OBJECT IS NOT LIST TYPE WHAT" << std::endl;
                    return false;
                }

                for (Py_ssize_t i = 0; i < PyList_Size(np_ret); i++) {
                    PyObject *elem = PyList_GetItem(np_ret, i);
                    //Each of these elements is an array of points?
                    if (!PyList_Check(elem)) {
                        std::cout << "LIST ELEMENT IS NOT A LIST" << std::endl;
                        return false;
                    }
                    for (Py_ssize_t j = 0; j < PyList_Size(elem); j++) {
                        geometry_msgs::PoseStamped point;
                        PyObject* py_point = PyList_GetItem(elem, 0);
                        float temp = PyFloat_AsDouble(py_point);
                        point.pose.position.x = temp;

                        py_point = PyList_GetItem(elem, 1);
                        temp = PyFloat_AsDouble(py_point);
                        point.pose.position.y = temp;

                        py_point = PyList_GetItem(elem, 2);
                        temp = PyFloat_AsDouble(py_point);
                        point.pose.orientation.x = temp;

                        py_point = PyList_GetItem(elem, 3);
                        temp = PyFloat_AsDouble(py_point);
                        point.pose.orientation.y = temp;

                        py_point = PyList_GetItem(elem, 4);
                        temp = PyFloat_AsDouble(py_point);
                        point.pose.orientation.z = temp;

                        py_point = PyList_GetItem(elem, 5);
                        temp = PyFloat_AsDouble(py_point);
                        point.pose.orientation.w = temp;

                        plan.push_back(point);
                    }
                    return true;
                }
                std::cout << std::endl;
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return false;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", "Global Planner Main");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "Global Planner");
        return 1;
    }
  return false;
 }
 };