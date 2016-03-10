#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpWireFrameSimulator.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpTime.h>
#include <visp/vpExponentialMap.h>
#include <ok_tools/okExperiment.h>
#include <visp/vpSubMatrix.h>
#include <ros/package.h>


using std::string;
using std::cout;
using std::endl;

int main ( int argc, char ** argv )
{

    // init and read config file
    vpIoTools::loadConfigFile(ros::package::getPath("windturb") + "/windturb.ini");
    string dataPath;    vpIoTools::readConfigVar ( "dataPath", dataPath );
    dataPath = vpIoTools::path(dataPath);
    string expID; vpIoTools::readConfigVar("expID", expID);
    vpIoTools::setBaseDir ( dataPath );
    vpIoTools::setBaseName (expID);

    bool useLines;vpIoTools::readConfigVar("useLines", useLines);
    bool useDiffAngles;vpIoTools::readConfigVar("useDiffAngles", useDiffAngles);
    bool useP0;vpIoTools::readConfigVar("useP0", useP0);
    bool use3D;vpIoTools::readConfigVar("use3D", use3D);
    bool rotating;vpIoTools::readConfigVar("rotating", rotating);
    double lambda;vpIoTools::readConfigVar("lambda", lambda);

    if(use3D)
        useP0 = useLines = useDiffAngles = rotating = false;

    // filenames
    vpIoTools::setBaseDir ( dataPath );
    vpIoTools::setBaseName ( expID );
    vpIoTools::addNameElement ( "3D", use3D );
    vpIoTools::addNameElement ( "P0", useP0 );
    vpIoTools::addNameElement ( "l", useLines );
    vpIoTools::addNameElement ( "dt", useDiffAngles );
    vpIoTools::addNameElement ( "rot", rotating );

    // init display
    vpImage<unsigned char> I ( 480,640,255 );
    I = 255;
    vpCameraParameters cam ( 1000,1000,320,240 );
    vpDisplayOpenCV display;
    display.init ( I, 100, 100,"Internal view" ) ;
    vpDisplay::setWindowPosition ( I, 200, 0 );
    vpDisplay::display ( I );

    // 3D lines
    vpLine line[4];
    vpPoint P[5];
    double offset = M_PI/6;
    line[0].setWorldCoordinates(1,0,0,0,0,1,0,0);   // x = y = 0
    P[1].setWorldCoordinates(0,0,-100);
    unsigned int i;
    for(i=0;i<3;++i)
    {
        line[i+1].setWorldCoordinates(0,1,0,0,cos(offset+i*2*M_PI/3), 0, sin(offset+i*2*M_PI/3),0);  // y = 0, x.cos(t) + z.sin(t) = 0
        P[i+2].setWorldCoordinates(-sin(offset+i*2*M_PI/3),0,cos(offset+i*2*M_PI/3));
    }

    // camera pose
    vpHomogeneousMatrix cMod(0,0,5,M_PI/2,0,0);
    vpHomogeneousMatrix cMo(10,10,50,M_PI/2,0,0.3);
    vpImagePoint ip1, ip2;
    vpColVector v(6);

    // features
    unsigned int m = 0;


    // central point
    vpFeaturePoint p, pd;
    P[0].changeFrame(cMod);
    P[0].project();

    if(useP0)
    {
        vpFeatureBuilder::create(pd, P[0]);
        m += 2;
    }

    // 2D lines
    vpFeatureLine l[4], ld[4];
    for(i=0;i<4;++i)
    {
        line[i].changeFrame(cMod);
        line[i].project();
        vpFeatureBuilder::create(ld[i], line[i]);

        if(useLines)
            m += 2;
        else if(i==0)
            m += 1;
    }

    cout << "m after lines: " << m << endl;

    // diff angles
    vpColVector dt(3), dtd(3), et;
    vpMatrix Ldt;
    unsigned int i2;
    if(useDiffAngles)
    {
        m += 3;
        for(i=0;i<2;++i)
        {
            i2 = ((i+1)%3) +1;
            cout << "i1: " << i+1 << ", i2: "<< i2 << endl;
            dtd[i] = ld[i2].error(ld[i+1])[1];
        }
    }
    vpMatrix L(m,6), Lt;
    vpColVector e(m);

    // exp
    okExperiment exp;
    vpPoseVector pose(cMod);
    vpColVector pose_v;
   // exp.save(pose_v, "err3D", exp.legendPose, "Pose error");
    exp.save3Dpose(pose_v, "pose", vpIoTools::getBaseName());
    exp.showMovingCamera();
    exp.setDesiredPose(pose);

    cout << "Looping" << endl;

    // loop
    while(!vpDisplay::getClick(I, false))
    {
        // save cMo
        pose.buildFrom(cMo);
        pose_v = pose;

        // rotating ?
        if(rotating)
        {
            offset += 0.01;
            for(i=0;i<3;++i)
            {
                line[i+1].setWorldCoordinates(0,1,0,0,cos(offset+i*2*M_PI/3), 0, sin(offset+i*2*M_PI/3),0);  // y = 0, x.cos(t) + z.sin(t) = 0
                P[i+2].setWorldCoordinates(-sin(offset+i*2*M_PI/3),0,cos(offset+i*2*M_PI/3));
            }
        }


        // update lines
        for(i=0;i<4;++i)
        {
            line[i].changeFrame(cMo);
            line[i].project();
            vpFeatureBuilder::create(l[i], line[i]);
        }
        // update point
        P[0].changeFrame(cMo);
        P[0].project();
        vpFeatureBuilder::create(p, P[0]);

        // control law
        if(use3D)
            v = vpExponentialMap::inverse(cMo*cMod.inverse());
        else
        {
            // need to append features
            m = 0;

            // central point
            if(useP0)
            {
                e[0] = p.error(pd)[0];
                e[1] = p.error(pd)[1];
                Lt = p.interaction();
                for(unsigned int j=0;j<6;++j)
                {
                    L[0][j] = Lt[0][j];
                    L[1][j] = Lt[1][j];
                }
                m += 2;
            }

            // 2D lines
            for(i=0;i<4;++i)
            {
                if(useLines)
                {
                    e[m] = l[i].error(ld[i])[0];
                    e[m+1] = l[i].error(ld[i])[1];
                    Lt = l[i].interaction();
                    for(unsigned int j=0;j<6;++j)
                    {
                        L[m][j] = Lt[0][j];
                        L[m+1][j] = Lt[1][j];
                    }
                    m += 2;
                }
                else if(i==0)
                {
                    e[m] = l[i].error(ld[i])[1];
                    for(unsigned int j=0;j<6;++j)
                        L[m][j] = l[i].interaction()[1][j];
                    m += 1;
                }
            }

            // diff angle
            if(useDiffAngles)
            {
                for(i=0;i<2;++i)
                {
                    i2 = ((i+1)%3) +1;
                    dt[i] = l[i2].error(l[i+1])[1];
                    e[m] = dt[i] - dtd[i];
                    Ldt = l[i2].interaction() - l[i+1].interaction();
                    for(unsigned int j=0;j<6;++j)
                    {
                        L[m][j] = Ldt[1][j];
                    }

                    m += 1;
                }
            }

            cout << "error: " << e.t() << endl;
            v = -lambda * L.pseudoInverse() * e;

            cout << "law: " << v.t() << endl;
        }


        vpDisplay::display ( I );
        for(i=0;i<5;++i)
        {
            P[i].changeFrame(cMo);
            P[i].project();
        }
        vpMeterPixelConversion::convertPoint(cam, P[0].get_x(), P[0].get_y(), ip1);

        for(i=1;i<5;++i)
        {
            vpMeterPixelConversion::convertPoint(cam, P[i].get_x(), P[i].get_y(), ip2);
            vpDisplay::displayLine(I, ip1, ip2, vpColor::red, 2);
        }

        vpDisplay::flush ( I );

        // update cMo
        cMo = vpExponentialMap::direct(v,0.01).inverse() * cMo;

        exp.update();
        vpTime::wait(0.01);
    }

    exp.writePlotHistory(vpIoTools::getFullName());

}
