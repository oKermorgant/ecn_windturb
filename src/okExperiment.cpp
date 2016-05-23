/****************************************************************************
 *
 * okExperiment.cpp 9 juin 2010 okermorg
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/


/*!
  \file okExperiment.cpp
  \brief Functions for saving to and loading from files, and lauching external plotter (Python script)
 */

#include <sstream>
#include <fstream>
#include <visp/vpIoTools.h>

#include <windturb/okExperiment.h>
#include <python2.7/Python.h>
#include <ros/package.h>

#define unused(x) ((void)(x == 1))

using std::stringstream;
using std::string;
using std::vector;
using std::cout;
using std::endl;


// init static variables
bool okExperiment::doPlot = true;
string okExperiment::timeUnit = "";
vpColVector okExperiment::timeVec = vpColVector();
string okExperiment::legendVelocity = "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]";
string okExperiment::legendPose = "[t_x,t_y,t_z,\\theta u_x,\\theta u_y,\\theta u_z]";
string okExperiment::unitVelocity = "[m/s,m/s,m/s,rad/s,rad/s,rad/s]";
string okExperiment::unitPose = "[m,m,m,rad,rad,rad]";
string okExperiment::boxGraph = "[[0,1],[1,2],[2,3],[3,0],[0,4],[1,5],[2,6],[3,7],[4,5],[5,6],[6,7],[7,4]]";

string okExperiment::scriptPath = ros::package::getPath("windturb") + "/scripts/python_plot";

// Generic variable
void okExperiment::saveVariable(const okDataType &dataType, vpColVector &v, const string name, const string legend, const string xlabel, const string ylabel, const bool &saveFile)
{

    stData newData;
    newData.dataType = dataType;
    newData.ptr_v = &v;
    newData.name = name;
    newData.legend = legend;
    newData.xlabel = xlabel;
    newData.ylabel = ylabel;
    newData.lineType = "";
    newData.units = "";
    newData.vSup = vpColVector();
    newData.desiredPose = vpColVector();
    newData.objectNodes = newData.fixedObjectNodes = vpMatrix();
    newData.objectGraph = newData.fixedObjectGraph = newData.fixedObjectColor = "";
    newData.saveFile = saveFile;

    // Store new structure
    varData.push_back(newData);
    lastData = &(varData.back());
    legendInit = false;
}

// Save iteration-based vector
void okExperiment::save(vpColVector &v, const string name, const string legend, const string ylabel, const bool &saveFile)
{
    saveVariable(okExperiment::ITERATION, v, name, legend, "iterations", ylabel, saveFile);
}

// Save time-based vector
void okExperiment::saveTimed(vpColVector &v, const string name, const string legend, const string ylabel, const bool &saveFile)
{

    saveVariable(okExperiment::TIME, v, name, legend, "time", ylabel, saveFile);
    if(timeUnit == "")
        std::cout << "okExperiment: warning, saving time-based value but does not have time" << std::endl;
}

// Save 3d pose or position
void okExperiment::save3Dpose(vpColVector &v, const string name, const string legend, const bool &saveFile)
{
    saveVariable(okExperiment::POSE, v, name, legend, "", "", saveFile); 
}

// Save normalized data
void okExperiment::saveNormalized(vpColVector &v, const string name, const string legend, const vpColVector &normBounds, const bool &saveFile)
{
    saveVariable(okExperiment::ITERATION_NORM, v, name, legend, "iterations", "", saveFile);
    addVector(normBounds);
}

// Save time-based normalized data
void okExperiment::saveTimedNormalized(vpColVector &v, const string name, const string legend, const vpColVector &normBounds, const bool &saveFile)
{
    saveVariable(okExperiment::TIME_NORM, v, name, legend, "iterations", "", saveFile);
    addVector(normBounds);
}

// **** End functions for new variables ****

// **** Functions to specify metadata for the last registered variable ****

// 3D plot: show moving camera
void okExperiment::showMovingCamera(const double &x, const double &y, const double &z)
{
    vpMatrix M(5,3);
    M[1][0] = x;    M[1][1] = -y;    M[1][2] = z;
    M[2][0] = -x;   M[2][1] = -y;    M[2][2] = z;
    M[3][0] = -x;   M[3][1] = y;     M[3][2] = z;
    M[4][0] = x;    M[4][1] = y;     M[4][2] = z;
    showMovingObject(M, "[[0,1],[0,2],[0,3],[0,4],[1,2],[2,3],[3,4],[4,1]]");
}

// 3D plot: show moving box
void okExperiment::showMovingBox(const double &x, const double &y, const double &z)
{
    vpMatrix M = buildBoxNodes(-x/2,-y/2,-z/2,x,y,z);
    showMovingObject(M, boxGraph);
}

// 3D plot: fixed 3D-rectangle
void okExperiment::showFixedBox(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM, const std::string &color)
{
    vpMatrix M = buildBoxNodes(xm, ym, zm, xM, yM, zM);
    showFixedObject(M, boxGraph, color);
}

// 3D plot: fixed 2D-rectangle on Z=0
void okExperiment::showFixedRectangle(const double &xm, const double &ym, const double &xM, const double &yM, const std::string &color)
{
    vpMatrix M(4,3);
    M[0][0] = xM;   M[0][1] = yM;
    M[1][0] = xM;   M[1][1] = ym;
    M[2][0] = xm;   M[2][1] = ym;
    M[3][0] = xm;   M[3][1] = yM;
    showFixedObject(M, "[[0,1],[1,2],[2,3],[3,0]]", color);
}

// 3D plot: build box nodes
vpMatrix okExperiment::buildBoxNodes(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM)
{
    vpMatrix M(8,3);
    M[0][0] = xm;    M[0][1] = ym;   M[0][2] = zm;
    M[1][0] = xM;     M[1][1] = ym;   M[1][2] = zm;
    M[2][0] = xM;     M[2][1] = yM;    M[2][2] = zm;
    M[3][0] = xm;    M[3][1] = yM;    M[3][2] = zm;
    M[4][0] = xm;    M[4][1] = ym;   M[4][2] = zM;
    M[5][0] = xM;     M[5][1] = ym;   M[5][2] = zM;
    M[6][0] = xM;     M[6][1] = yM;    M[6][2] = zM;
    M[7][0] = xm;    M[7][1] = yM;    M[7][2] = zM;
    return M;
}


// Updates all saved vectors
void okExperiment::update()
{
    unsigned int i,j;

    if(!legendInit)
    {
        for(i=0;i<varData.size();++i)
        {
            stData& data(varData[i]);
            // check legend if custom: x -> [x_1,x_2,...,x_n]
            // if 3D, assume that legend may be without brackets : x -> [x]
            if(data.legend != "" && data.legend.compare(0,1,"[") != 0)
            {
                stringstream s;
                s << "[";
                if(data.dataType == POSE)
                    s << data.legend;
                else
                {
                    for(j=0;j<data.v().getRows();++j)
                    {
                        s << data.legend << "_" << j+1;
                        if(j != data.v().getRows()-1)
                            s << ",";
                    }
                }
                s << "]";
                data.legend = s.str();
            }
        }
        legendInit = true;
    }

    // update time vector if needed
    if(timeUnit != "")
    {
        vpColVector tv(1);
        tv = *time;
        timeVec.stack(tv);
    }

    // update history matrix
    for(i=0;i<varData.size();++i)
        varData[i].vHist.stack(vpColVector::stack(varData[i].v(), varData[i].vSup).t());
}

// Saves and plot data (will not plot if doPlot == false)
void okExperiment::writeData(stData &data, const string &filename)
{
    unsigned int i;
    // build file header
    std::string header;

    if(data.vHist.getRows() == 0)
    {
        std::cout << "Could not plot " << filename << " : matrix has no rows" << std::endl;
        return;
    }

    if(data.vHist.getCols() == 0)
    {
        std::cout << "Could not plot " << filename << " : matrix has no cols" << std::endl;
        return;
    }

    cout << "Writing to " << filename << endl;

    // data type
    header += "dataType: ";
    switch(data.dataType)
    {
    case ITERATION:
    case ITERATION_NORM:
        header +=  "iteration-based\n";
        break;
    case TIME:
    case TIME_NORM:
        header +=  "time-based\n";
        break;
    case okExperiment::POSE:
        header +=  "3D pose\n";
    }

    // type-related metadata
    if(data.dataType == POSE)
    {
        std::string tmp = "";
        if(data.desiredPose.getRows() != 0)
            tmp +=  "  desiredPose: " + singleLineYAML(data.desiredPose.t()) + "\n";
        if(data.objectNodes.getCols() != 0 && data.objectNodes.getRows() != 0 && data.objectGraph != "")
        {
            tmp +=  "  nodes: " + singleLineYAML(data.objectNodes) + "\n";
            tmp +=  "  graph: " + data.objectGraph + "\n";
        }
        if(tmp != "")
            header += "movingObject:\n" + tmp;
        if(data.fixedObjectNodes.getCols() != 0 && data.fixedObjectNodes.getRows() != 0 && data.fixedObjectGraph != "")
        {
            header += "fixedObject:\n";
            header +=  "  nodes: " + singleLineYAML(data.fixedObjectNodes) + "\n";
            header +=  "  graph: " + data.fixedObjectGraph + "\n";
            if(data.fixedObjectColor != "")
                header += "  color: " + data.fixedObjectColor + "\n";
        }
    }

    // axes labels
    if(data.xlabel != "") header +=  "xlabel: " + data.xlabel  + "\n";
    if(data.ylabel != "") header +=  "ylabel: " + data.ylabel  + "\n";

    // column-wise metadata
    if(data.units != "")  header +=  "units: " + data.units  + "\n";
    if(data.dataType == TIME || data.dataType == TIME_NORM)
        header +=  "time unit: " + timeUnit  + "\n";
    // append standard bounds linetypes before writing
    if(data.dataType == TIME_NORM || data.dataType == ITERATION_NORM)
    {
        string lineNorm;
        for(i=0;i<data.vSup.getRows();++i)
        {
            if(i < 2)
                lineNorm += "k-";
            else
                lineNorm += "k--";
            if(i != data.vSup.getRows()-1)
                lineNorm += ", ";
        }
        lineNorm += "]";

        if(data.lineType != "")
        {
            lineNorm = ", " + lineNorm;
            for(i=data.lineType.length()-1;i!= 0;--i)
                lineNorm = data.lineType[i] + lineNorm;
        }
        data.lineType = "[" + lineNorm;
    }
    if(data.lineType != "")  header +=  "lineType: " + data.lineType  + "\n";
    if(data.legend != "")   header +=  "legend: " + data.legend  + "\n";

    // actual data
    if(data.dataType == TIME || data.dataType == TIME_NORM)
        data.vHist = vpMatrix::juxtaposeMatrices(timeVec, data.vHist);
    vpMatrix::saveMatrixYAML(filename, data.vHist, header.substr(0, header.length()-1).c_str());
}


// Saves and plots all history files (will not plot if doPlot == false)
void okExperiment::writePlotHistory(const string &basename)
{
    string filename;
    for(unsigned int i=0;i<varData.size();++i)
    {
        if(varData[i].saveFile)
            filename = basename + "_" + varData[i].name + ".txt";
        else
        {
            char ctpl[] = "/tmp/okExp.XXXXXX";
            unused(mkstemp(ctpl));	// randomizing name
            filename = string(ctpl);
        }
        writeData(varData[i], filename);
        plotFile(filename);
    }
}

// writes a matrix into a single line YAML format
string okExperiment::singleLineYAML(const vpMatrix &M)
{
    stringstream ss;
    unsigned int i,j;

    ss << "[";
    for(i=0;i<M.getRows();++i)
    {
        ss << "[";
        for(j=0;j<M.getCols()-1;++j)
            ss << M[i][j] << ", ";
        ss << M[i][j] << "]";
        if(i != M.getRows()-1)
            ss << ",";
    }
    ss << "]";
    return ss.str();
}

// ***** Plotting functions

// Plot a file
void okExperiment::plotFile(const string &filename)
{
    // if doPlot == false, actually do not plot anything
    if(!doPlot)
        return;

    // Old school : launch external application
    string cmdline = "python " + scriptPath + " " + filename;
    cmdline += " ";
    cout << "executing "<< cmdline << endl;
    unused(system(cmdline.c_str()));
}

// *** static functions to write and/or plot a basic matrix

// Dump matrix to file
void okExperiment::writeMatrix(const vpMatrix &M, const string &filename, const string &legend)
{
    // build data structure according to M
    stData data;
    data.dataType = ITERATION;
    data.vHist = M;
    data.legend = legend;

    writeData(data, filename);
}

// Plot a matrix via given file
void okExperiment::writePlotMatrix(const vpMatrix &M, const string &filename, const string &legend)
{
    writeMatrix(M, filename, legend);
    plotFile(filename);
}

// Plot a matrix via temporary file
void okExperiment::plotMatrix(const vpMatrix &M, const string &legend)
{
    // temporary name, then plot and indicate to Python to erase the file afterwards
    char ctpl[] = "/tmp/vpPlot.XXXXXX";
    unused(mkstemp(ctpl));	// randomizing name
    string filename(ctpl);
    // plot the file
    writePlotMatrix(M, filename, legend);
}

// **** end plotting functions


// **** Related to legends

string okExperiment::legendFeatPoint(const unsigned int &n)
{
    stringstream ss;
    ss << "[";
    unsigned int i;
    for(i=0;i<n-1;++i)
        ss << "x_" << i+1 << ", y_" << i+1 << ", ";
    ss << "x_" << i+1 << ", y_" << i+1 << "]";
    return ss.str();
}

// **** end legends

// special lecture Rxyz

void readConfigVar_PoseRxyz(const string &s, vpHomogeneousMatrix &M)
{
    vpColVector v(6);
    vpIoTools::readConfigVar(s, v);
    vpTranslationVector t(v[0], v[1], v[2]);
    vpRxyzVector r(v[3], v[4], v[5]);
    vpRotationMatrix R(r);
    M.buildFrom(t, R);
}
