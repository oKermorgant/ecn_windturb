/****************************************************************************
 *
 * okExperiment.h 9 juin 2010 okermorg
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is NOT part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 *
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/

#ifndef OKEXPERIMENT_H_
#define OKEXPERIMENT_H_


/*!
  \file okExperiment.h
  \brief functions to plot specific files
 */

#include <iostream>

#include <string>
#include <vector>
#include <visp/vpMatrix.h>
#include <visp/vpConfig.h>
#include <visp/vpRowVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMath.h>

class VISP_EXPORT okExperiment
{
protected:
    enum okDataType
    {
        ITERATION,
        TIME,
        POSE,
        ITERATION_NORM,
        TIME_NORM
    };

    // structure containing everything related to a variable: metadata and data
    struct stData {
        // global to all types of data
        vpColVector* ptr_v;  // pointer to saved vector
        vpColVector vSup;   // constant vector to append
        vpMatrix vHist;     // matrix of saved vectors
        std::string name;   // name of variable (will go in filename)

        // related to data type and metadata
        okDataType dataType;
        std::string legend;
        std::string xlabel;
        std::string ylabel;
        std::string units;
        std::string lineType;

        // Specific to 3D plot
        // Moving object with (or without) desired pose
        vpMatrix objectNodes;
        std::string objectGraph;
        vpColVector desiredPose;
        // Fixed object
        vpMatrix fixedObjectNodes;
        std::string fixedObjectGraph;
        std::string fixedObjectColor;

        bool saveFile;  // if false, will save the file in /tmp

        // get actual v
        inline const vpColVector v() {return *ptr_v;}
    };

    // static variables
    static bool doPlot;
    static std::string scriptPath;

    // related to high-level functions
    double * time;
    static std::string timeUnit;
    static vpColVector timeVec;
    std::vector<stData> varData;
    stData* lastData;
    bool legendInit;

public:
    inline okExperiment()
    {
        varData.clear();
        legendInit = false;
    }

    // High-level functions
    // Set time
    inline void setTime(double &t, const std::string &tUnit) {time = &t;timeUnit = tUnit;}
    // **** Functions to add new variables to be saved ****
    // Generic function
    void saveVariable(const okDataType &dataType, vpColVector &v, const std::string name, const std::string legend, const std::string xlabel, const std::string ylabel, const bool &saveFile = true);
    // Save iteration-based vector
    void save(vpColVector &v, const std::string name, const std::string legend, const std::string ylabel, const bool &saveFile = true);
    // Save time-based vector
    void saveTimed(vpColVector &v, const std::string name, const std::string legend, const std::string ylabel, const bool &saveFile = true);
    // Save 3d pose or position
    void save3Dpose(vpColVector &v, const std::string name, const std::string legend, const bool &saveFile = true);
    // Save normalized data
    void saveNormalized(vpColVector &v, const std::string name, const std::string legend, const vpColVector &normBounds, const bool &saveFile = true);
    // Save time-based normalized data
    void saveTimedNormalized(vpColVector &v, const std::string name, const std::string legend, const vpColVector &normBounds, const bool &saveFile = true);
    // **** End functions for new variables ****

    // **** Functions to specify metadata for the last registered variable ****
    // Vector to append to data
    inline void addVector(const vpColVector &v) {lastData->vSup = v;}
    // Double to append to data
    inline void addDouble(const double &v) {lastData->vSup.stack(v);}
    // Units
    inline void setUnits(const std::string units) {lastData->units = units;}
    // Line types
    inline void setLineType(const std::string lineType) {lastData->lineType = lineType;}
    // 3D plot: desired pose
    inline void setDesiredPose(const vpColVector pose) {lastData->desiredPose = pose;}
    // 3D plot: show camera
    void showMovingCamera(const double &x = 1.5, const double &y = 1, const double &z = 4);
    // 3D plot: show box
    void showMovingBox(const double &x = 10, const double &y = 5, const double &z = 3);
    // 3D plot: custom object with a (nx3) matrix
    inline void showMovingObject(const vpMatrix &M, const std::string &graph) {lastData->objectNodes = M;lastData->objectGraph = graph;}
    // 3D plot: fixed object (related to object frame)
    inline void showFixedObject(const vpMatrix &M, const std::string &graph, const std::string &color = "") {lastData->fixedObjectNodes = M;lastData->fixedObjectGraph = graph;lastData->fixedObjectColor = color;}
    // 3D plot: fixed 3D-box
    void showFixedBox(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM, const std::string &color = "");
    // 3D plot: fixed 2D-rectangle on Z=0
    void showFixedRectangle(const double &xm, const double &ym, const double &xM, const double &yM, const std::string &color = "");
    // **** End metadata functions ****

    // Updates all saved variables
    void update();
    // Saves history files and plot
    void writePlotHistory(const std::string &basename);

    // Low-level static functions
    // Build 3D box nodes depending on dimensions
    vpMatrix buildBoxNodes(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM);
    // Builds YAML-proof single-lined matrix
    static std::string singleLineYAML(const vpMatrix &M);

    // Save structure to file and plot
    static void writeData(stData &data, const std::string &filename);

    // Plot a file
    static void plotFile(const std::string &filename);
    // Dump matrix to file
    static void writeMatrix(const vpMatrix &M, const std::string &filename, const std::string &legend = "");
    // Plot a matrix via given file
    static void writePlotMatrix(const vpMatrix &M, const std::string &filename, const std::string &legend = "");
    // Plot a matrix via temporary file
    static void plotMatrix(const vpMatrix &M, const std::string &legend = "");

    // Tuning functions
    inline static void setDoPlot(const bool &b) {doPlot = b;}
    inline static void setScriptPath(const std::string &s) {scriptPath = s;}

    // usual legends
    static std::string legendVelocity,legendPose;
    static std::string legendFeatPoint(const unsigned int &n=4);

    // usual units
    static std::string unitVelocity, unitPose;
    // box graph
    static std::string boxGraph;

};


// afficher le contenu d'une variable
inline void printvar(const std::string &leg, vpMatrix M) {std::cout << leg << " : ";M.printSize();std::cout << std::endl << M << std::endl;}
inline void printvar(const std::string &leg, const double &s) {std::cout << leg << " : " << s << std::endl;}
inline void printvar(const std::string &leg, const std::string &s = "") {std::cout << leg; if(s!= "") std::cout << " : " << s; std::cout << std::endl;}
inline void printvar(const std::string &leg, const int &s) {std::cout << leg << " : " << s << std::endl;}
inline void printvar(const std::string &leg, const unsigned int &s) {std::cout << leg << " : " << s << std::endl;}
inline void printvar(const std::string &leg, const bool &b) {if(b == true) std::cout << leg << " : true" << std::endl;else std::cout << leg << " : false" << std::endl;}
inline void printvar(const std::string &leg, const vpHomogeneousMatrix &M) {vpPoseVector pose(M);printvar(leg, pose.t());}

// lecture spéciale pose Rxyz
void readConfigVar_PoseRxyz(const std::string &s, vpHomogeneousMatrix &M);

// print debug
inline void printDb(const std::string &s, const unsigned int &dbLvl, const unsigned int &db = 0)
{if(db >= dbLvl) std::cout << s << std::endl;}

#endif /* OKEXPERIMENT_H_ */
