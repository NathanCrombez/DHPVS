/*
 * DHPVS: Dual-Hemispherical Photometric Visual Servoing
 * Author: Nathan Crombez (nathan.crombez@utbm.fr)
 * Institution: CIAD-UTBM
 * Date: January 2024
 */

////ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

////BOOST
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

////ViSP
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpExponentialMap.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageTools.h>
#include <visp/vpPlot.h>
#include <visp3/core/vpImageFilter.h>



////ROBOT
ros::Publisher robotVelocityPub, robotPosePub;

////CAMERA
typedef struct {
    int height, width;
    float au, av;
    float u0, v0;
    float xi;
    vpHomogeneousMatrix cMf;
} cameraParameters;
cameraParameters rightCameraParameters, leftCameraParameters;

////IMAGES
vpImage<unsigned char> rightI, leftI, rightId, leftId, rightIdiff, leftIdiff;

////DISPLAYS
vpDisplayX rightDispI, leftDispI, rightDispId, leftDispId, rightDispIdiff, leftDispIdiff;
vpPlot plot;

////VISUAL SERVOING
vpPoseVector desiredRobotPose, currentRobotPose, initialRobotPose;
vpMatrix L;
vpColVector e, v;
double Z;
int iter;
double gain;
bool vsStarted;

////OTHERS
ros::Time t;
bool verbose;
int qSize;

////Main VS loop (callback)
void camerasImageRobotPoseCallback(const sensor_msgs::Image::ConstPtr &rightImsg, const sensor_msgs::Image::ConstPtr &leftImsg, const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg);
////DHP error
void computeDHPErrorVector(vpImage<unsigned char> rightI, vpImage<unsigned char> rightId, vpImage<unsigned char> leftI, vpImage<unsigned char> leftId, vpColVector &e);
////DHP interaction matrix
void computeDHPInteractionMatrix(vpImage<unsigned char> rightI, cameraParameters rightCamParam, vpImage<unsigned char> leftI, cameraParameters leftCamParam, vpMatrix &L, double Z=1.0);

////Camera's poses initialization
void cameraPosesInitialization();

////VISP <--> ROS Messages
vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i, std::string frame_o);
geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity);
cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg);


int main(int argc, char **argv){
    ros::init(argc, argv, "DHPVS", ros::init_options::NoSigintHandler);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle nh("~");

    qSize = 30;
    vsStarted = false;
    v.resize(6);

    ////Get parameters
    verbose = nh.param("verbose", 1);
    gain = nh.param("gain", 1.0);
    Z = nh.param("Z", 1.0);

    ////Get right and left cameras intrinsic parameters
    rightCameraParameters = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/dhpvs/right_camera/camera_info", nh));
    leftCameraParameters = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/dhpvs/left_camera/camera_info", nh));
    ////Get right and left cameras extrinsic parameters
    rightCameraParameters.cMf = vpHomogeneousMatrixFromROSTransform("/flange", "/right_camera");
    leftCameraParameters.cMf = vpHomogeneousMatrixFromROSTransform( "/flange", "/left_camera");

    ////Robot velocities publisher
    robotVelocityPub = nh.advertise<geometry_msgs::Twist>("/dhpvs/robot/set_velocity", qSize);

    ////Cameras and robot synchronizer
    message_filters::Subscriber<sensor_msgs::Image> rightCameraSub, leftCameraSub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> robotPoseSub;
    rightCameraSub.subscribe(nh, "/dhpvs/right_camera/image_raw", qSize);
    leftCameraSub.subscribe(nh, "/dhpvs/left_camera/image_raw", qSize);
    robotPoseSub.subscribe(nh,"/dhpvs/robot/get_pose", qSize);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>> camerasSynchronizer(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>(qSize), rightCameraSub, leftCameraSub, robotPoseSub);
    camerasSynchronizer.registerCallback(boost::bind(&camerasImageRobotPoseCallback, _1, _2, _3));

    ////Image displayers
    rightI.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftI.resize(leftCameraParameters.height, leftCameraParameters.width);
    rightId.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftId.resize(leftCameraParameters.height, leftCameraParameters.width);
    rightIdiff.resize(rightCameraParameters.height, rightCameraParameters.width);
    leftIdiff.resize(leftCameraParameters.height, leftCameraParameters.width);
    leftDispI.init(leftI, 0, 27, "LEFT I");
    leftDispId.init(leftId, 0, 27+37+leftCameraParameters.height, "LEFT I*");
    leftDispIdiff.init(leftIdiff, 0,  27+(37+leftCameraParameters.height)*2, "LEFT DIFF");
    rightDispI.init(rightI, rightCameraParameters.width, 27, "RIGHT I");
    rightDispId.init(rightId, rightCameraParameters.width,  27+37+rightCameraParameters.height, "RIGHT I*");
    rightDispIdiff.init(rightIdiff, rightCameraParameters.width,  27+(37+rightCameraParameters.height)*2, "RIGHT DIFF");

    ////Plots
    plot.init(4,(37+rightCameraParameters.height)*3-37, rightCameraParameters.width, rightCameraParameters.width*2,27);
    plot.initGraph(0,1); plot.setTitle(0,"Feature error"); plot.setLegend(0,0,"error");
    plot.initGraph(1,6); plot.setTitle(1,"Velocities");
    plot.setLegend(1,0,"v_x");     plot.setLegend(1,1,"v_y");     plot.setLegend(1,2,"v_z");
    plot.setLegend(1,3,"w_x");     plot.setLegend(1,4,"w_y");     plot.setLegend(1,5,"w_z");
    plot.initGraph(2,3); plot.setTitle(2,"Translation error");
    plot.setLegend(2,0,"dtx_x");     plot.setLegend(2,1,"dt_y");     plot.setLegend(2,2,"dt_z");
    plot.initGraph(3,3); plot.setTitle(3,"Orientation error");
    plot.setLegend(3,0,"dw_x");     plot.setLegend(3,1,"dw_y");     plot.setLegend(3,2,"dw_z");

    ////Camera desired and initial pose initialization
    cameraPosesInitialization();

    ////Actually start DHPVS
    iter=0;
    vsStarted = true;
    t = ros::Time::now();
    ros::spin();
}

void cameraPosesInitialization(){
    for(int i=3;i>0;i--){
        ROS_WARN("%d", i);
        sleep(1);
    }

    ROS_WARN("Move the camera to the desired pose then click on LEFT I");
    do{
        rightId = rightI;
        leftId = leftI;
        vpDisplay::display(rightId); vpDisplay::flush(rightId);
        vpDisplay::display(leftId);  vpDisplay::flush(leftId);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    desiredRobotPose = currentRobotPose;

    ROS_WARN("Move the camera to the initial pose then click on LEFT I");
    do{
        vpImageTools::imageDifference(leftI, leftId, leftIdiff);
        vpImageTools::imageDifference(rightI, rightId, rightIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    initialRobotPose = currentRobotPose;

    ROS_WARN("Cick on LEFT I to start VS");
    do{
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
}

void camerasImageRobotPoseCallback(const sensor_msgs::Image::ConstPtr &rightImsg, const sensor_msgs::Image::ConstPtr &leftImsg, const geometry_msgs::PoseStamped::ConstPtr &robotPoseMsg) {
    ////ROS to VISP
    rightI = visp_bridge::toVispImage(*rightImsg);
    leftI = visp_bridge::toVispImage(*leftImsg);
    currentRobotPose = vpPoseVector(visp_bridge::toVispHomogeneousMatrix(robotPoseMsg->pose));

    if (vsStarted) {
        if(robotPoseMsg->header.stamp<=t)   //Prevent ROS synchro issues
             return;

        ////Displays
        vpImageTools::imageDifference(rightI, rightId,rightIdiff);
        vpImageTools::imageDifference(leftI, leftId,leftIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);

        ////Compute interaction matrix
        computeDHPInteractionMatrix(rightI,rightCameraParameters,leftI,leftCameraParameters,L, Z);
        ////Compute error vector
        computeDHPErrorVector(rightI,  rightId,  leftI, leftId, e);
        ////Compute Gauss-newton control law
        v = -gain * L.pseudoInverseEigen3() * e; //Velocities expressed in the robot's flange

        ////Send velocity to robot
        robotVelocityPub.publish(geometryTwistFromvpColVector(v));

        t = ros::Time::now();

        ////PLOTS
        plot.plot(0,0,iter, e.sumSquare());
        for(int i=0;i<3;i++)
            plot.plot(2,i,iter,currentRobotPose[i]-desiredRobotPose[i]);
        for(int i=0;i<3;i++)
            plot.plot(3, i, iter, vpMath::deg(currentRobotPose[i+3]-desiredRobotPose[i+3]));
        for(int i=0;i<6;i++)
            plot.plot(1,i,iter, v[i]);

        ////VERBOSE
        if(verbose){
            ROS_DEBUG("Iteration: %d", iter);
            ROS_DEBUG("Velocities: %f %f %f %f %f %f", v[0], v[1], v[2], v[3], v[4], v[5]);
            ROS_DEBUG("Current Pose: %f %f %f %f %f %f", currentRobotPose[0], currentRobotPose[1], currentRobotPose[2], currentRobotPose[3], currentRobotPose[4], currentRobotPose[5]);
            ROS_DEBUG("Desired Pose: %f %f %f %f %f %f", desiredRobotPose[0], desiredRobotPose[1], desiredRobotPose[2], desiredRobotPose[3], desiredRobotPose[4], desiredRobotPose[5]);
            ROS_DEBUG("Error Pose: %f %f %f %f %f %f", fabs(currentRobotPose[0]-desiredRobotPose[0]),
                                                       fabs(currentRobotPose[1]-desiredRobotPose[1]),
                                                       fabs(currentRobotPose[2]-desiredRobotPose[2]),
                                                       vpMath::deg(fabs(currentRobotPose[3]-desiredRobotPose[3])),
                                                       vpMath::deg(fabs(currentRobotPose[4]-desiredRobotPose[4])),
                                                       vpMath::deg(fabs(currentRobotPose[5]-desiredRobotPose[5])));
            ROS_DEBUG("Photometric error: %f", e.sumSquare());
        }

        iter++;
    }
}

void computeDHPErrorVector(vpImage<unsigned char> rightI, vpImage<unsigned char> rightId, vpImage<unsigned char> leftI, vpImage<unsigned char> leftId, vpColVector &e){
    e.resize((rightI.getRows()-20)*(rightI.getCols()-20)  + (leftI.getRows()-20)*(leftI.getCols()-20) );
    unsigned char *ptrrightI = rightI.bitmap;
    unsigned char *ptrrightId = rightId.bitmap;
    unsigned char *ptrleftI = leftI.bitmap;
    unsigned char *ptrleftId = leftId.bitmap;

    int i=0;
    ////RIGHT CAMERA
    for(int v=10;v<rightI.getRows()-10;v++){
        for(int u=10;u<rightI.getCols()-10;u++,i++) {
            e[i] = (double)(ptrrightI[u + v * rightI.getCols()])-(double)(ptrrightId[u + v * rightI.getCols()]);
        }
    }
    ////LEFT CAMERA
    for(int v=10;v<leftI.getRows()-10;v++){
        for(int u=10;u<leftI.getCols()-10;u++,i++) {
            e[i] = (double)(ptrleftI[u + v * leftI.getCols()])-(double)(ptrleftId[u + v * leftI.getCols()]);
        }
    }
}


void computeDHPInteractionMatrix(vpImage<unsigned char> rightI, cameraParameters rightCamParam, vpImage<unsigned char> leftI, cameraParameters leftCamParam, vpMatrix &L, double Z){
    L.clear();
    L.resize((rightI.getRows()-20)*(rightI.getCols()-20)  + (leftI.getRows()-20)*(leftI.getCols()-20) , 6);
    vpVelocityTwistMatrix V;
    vpRowVector dIdx(2);
    vpMatrix dxdr(2,6, 0.0);
    vpRowVector dIdr(6);

    ////RIGHT CAMERA
    V.buildFrom(rightCamParam.cMf.getRotationMatrix());
    int i=0;
    for(int v=10;v<rightI.getRows()-10;v++) {
        for (int u=10;u<rightI.getCols()-10;u++,i++) {
            double xi = rightCamParam.xi;
            dIdx[0] = -rightCamParam.au * vpImageFilter::derivativeFilterX(rightI, v, u);
            dIdx[1] = -rightCamParam.au * vpImageFilter::derivativeFilterY(rightI, v, u);
            double x = (double)((1.0*u-rightCamParam.u0)/rightCamParam.au);
            double y = (double)((1.0*v-rightCamParam.v0)/rightCamParam.av);
            double srac = 1.0+(1.0-xi*xi)*(x*x + y*y);
            dxdr.resize(2,6,true);
            if(srac>=0.0) {
                double fact = (xi + sqrt(srac)) / (x*x + y*y + 1.0);
                double Zs = fact - xi;
                if(Zs !=0){
                    double rho = Z/Zs;
                    dxdr[0][0] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*y*y/rho) ;
                    dxdr[0][1] = xi*x*y/rho;
                    dxdr[0][2] = x * ((rho+xi*Z) / (rho*(Z + xi*rho)));
                    dxdr[0][3] = x*y;
                    dxdr[0][4] = -( x*x + (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) );
                    dxdr[0][5] = y;
                    dxdr[1][0] = xi*x*y/rho;
                    dxdr[1][1] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*x*x/rho );
                    dxdr[1][2] = y * ( (rho+xi*Z)/(rho*(Z + xi*rho)) );
                    dxdr[1][3] = ( (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) ) + y*y;
                    dxdr[1][4] = -x*y;
                    dxdr[1][5] = -x;
                }
            }
            dIdr = dIdx * dxdr;
            L.insert(dIdr*V, i, 0);
        }
    }
    ////LEFT CAMERA
    V.buildFrom(leftCamParam.cMf.getRotationMatrix());
    for(int v=10;v<leftI.getRows()-10;v++) {
        for (int u=10;u<leftI.getCols()-10;u++,i++) {
            double xi = leftCamParam.xi;
            dIdx[0] = -leftCamParam.au * vpImageFilter::derivativeFilterX(leftI, v, u);
            dIdx[1] = -leftCamParam.au * vpImageFilter::derivativeFilterY(leftI, v, u);
            double x = (double)((1.0*u-leftCamParam.u0)/leftCamParam.au);
            double y = (double)((1.0*v-leftCamParam.v0)/leftCamParam.av);
            double srac = 1.0+(1.0-xi*xi)*(x*x + y*y);
            dxdr.resize(2,6,true);
            if(srac>=0.0) {
                double fact = (xi + sqrt(srac)) / (x*x + y*y + 1.0);
                double Zs = fact - xi;
                if(Zs !=0){
                    double rho = Z/Zs;
                    dxdr[0][0] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*y*y/rho) ;
                    dxdr[0][1] = xi*x*y/rho;
                    dxdr[0][2] = x * ((rho+xi*Z) / (rho*(Z + xi*rho)));
                    dxdr[0][3] = x*y;
                    dxdr[0][4] = -( x*x + (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) );
                    dxdr[0][5] = y;
                    dxdr[1][0] = xi*x*y/rho;
                    dxdr[1][1] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*x*x/rho );
                    dxdr[1][2] = y * ( (rho+xi*Z)/(rho*(Z + xi*rho)) );
                    dxdr[1][3] = ( (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) ) + y*y;
                    dxdr[1][4] = -x*y;
                    dxdr[1][5] = -x;
                }
            }
            dIdr = dIdx * dxdr;
            L.insert(dIdr*V, i, 0);
        }
    }
}


/**************************************/
/**************ROS <-> VISP*************/
/**************************************/

vpHomogeneousMatrix vpHomogeneousMatrixFromROSTransform(std::string frame_i, std::string frame_o){
    geometry_msgs::Pose oMi;
    tf::StampedTransform oMi_tf;
    tf::TransformListener listener;
    listener.waitForTransform(frame_o, frame_i, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(frame_o, frame_i, ros::Time(0), oMi_tf);
    tf::poseTFToMsg(oMi_tf, oMi);
    return visp_bridge::toVispHomogeneousMatrix(oMi);
}

geometry_msgs::Twist geometryTwistFromvpColVector(vpColVector vpVelocity){
    geometry_msgs::Twist geoVelocity;
    geoVelocity.linear.x = vpVelocity[0];
    geoVelocity.linear.y = vpVelocity[1];
    geoVelocity.linear.z = vpVelocity[2];
    geoVelocity.angular.x = vpVelocity[3];
    geoVelocity.angular.y = vpVelocity[4];
    geoVelocity.angular.z = vpVelocity[5];
    return geoVelocity;
}

cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg){
    cameraParameters cam;
    cam.height = msg.height;
    cam.width = msg.width;
    cam.au = msg.K[0];
    cam.av = msg.K[4];
    cam.u0 = msg.K[2];
    cam.v0 = msg.K[5];
    cam.xi = msg.D[4];
    return cam;
}






