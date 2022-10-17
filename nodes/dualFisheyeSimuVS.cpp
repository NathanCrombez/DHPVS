/*
 * Dual Fisheye Visual Servoing
 * Author: Nathan Crombez (CIAD-UTBM)
 * Date: September 2022
 */

////ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

////ViSP
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpExponentialMap.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageTools.h>
#include <visp/vpPlot.h>


#include "../include/dualFisheyeVS/dualFisheyeVS.h"

class dualFisheyeVS{
private:
    ros::NodeHandle nh;
    int qSize;
    message_filters::Subscriber<sensor_msgs::Image> rightCameraSub, leftCameraSub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> cameraPoseSub;
    ros::Publisher cameraPosePub;
    ros::ServiceClient rightCameraInfoPub, leftCameraInfoPub;
    sensor_msgs::SetCameraInfo cameraInfo;

    std::string rightCameraTopic, leftCameraTopic;
    std::string rightCameraInfoTopic, leftCameraInfoTopic;

    vpImage<unsigned char> rightI, leftI, rightId, leftId, rightIdiff, leftIdiff;
    cameraParameters rightCameraParam, leftCameraParam;

    std::vector<luminanceFeature> rightS, rightSd, leftS, leftSd, s, sd;
    vpHomogeneousMatrix cMo, rMc, lMc;
    vpPoseVector cdpo, cpo;

    vpMatrix L, cLr, cLl;
    vpColVector e, v;
    float gain;
    int iter;

    vpDisplayX rightDispI, leftDispI, rightDispId, leftDispId, rightDispIdiff, leftDispIdiff;

    vpPlot plot;

    bool vsStarted;



public:
    dualFisheyeVS(int argc, char **argv);

    void camerasImageCallback(const sensor_msgs::Image::ConstPtr& frontImage, const sensor_msgs::Image::ConstPtr& backImage, const geometry_msgs::PoseStamped::ConstPtr& cameraPose);
    void cameraPosesInitialization();
    cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg);
    geometry_msgs::Pose getGeoPoseFromTransform(std::string frame_i, std::string frame_o);
    vpHomogeneousMatrix getvpMatrixfromTransform(std::string frame_i, std::string frame_o);

};


dualFisheyeVS::dualFisheyeVS(int argc, char **argv) : nh(){
    qSize = 10;
    vsStarted = false;
    ////Visual servoing parameters
    gain = nh.param("/gain", 0.75);     //TODO :
    rightCameraInfoTopic = nh.param<std::string>("/rightCameraTopic", "/camera_front/camera_info");
    leftCameraInfoTopic = nh.param<std::string>("/leftCameraTopic", "/camera_back/camera_info");

    ////Get/Set right and left camera intrinsic parameters
    rightCameraParam = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightCameraInfoTopic, nh));
    leftCameraParam = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(leftCameraInfoTopic, nh));
    cameraInfo.request.camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightCameraInfoTopic, nh);
    rightCameraInfoPub = nh.serviceClient<sensor_msgs::SetCameraInfo>("/camera_front/set_camera_info");
    leftCameraInfoPub = nh.serviceClient<sensor_msgs::SetCameraInfo>("/camera_back/set_camera_info");
    cameraInfo.request.camera_info.width=640/2;
    cameraInfo.request.camera_info.height=480/2;
    cameraInfo.request.camera_info.K[0]=319.5660095214844/2.0;
    cameraInfo.request.camera_info.K[2]=320.0/2.0;
    cameraInfo.request.camera_info.K[4]=239.6745147705078/2.0;
    cameraInfo.request.camera_info.K[5]=240/2.0;
    cameraInfo.request.camera_info.D[0]=0.800000011920929;
    rightCameraInfoPub.call(cameraInfo);
    leftCameraInfoPub.call(cameraInfo);


    ////Get/Set right and left camera extrinsic parameters
    cameraPoseSub.subscribe(nh, "/camera_link_controller/pose", qSize);
    cameraPosePub = nh.advertise<geometry_msgs::Pose>("/camera_link_controller/set_pose", qSize);
    rMc = getvpMatrixfromTransform("/camera_link", "/camera_front_optical_frame");
    lMc = getvpMatrixfromTransform("/camera_link", "/camera_back_optical_frame");

    ////Right and left camera subscribers
    rightCameraTopic = nh.param<std::string>("/rightCameraTopic", "/camera_front/image_raw");
    leftCameraTopic = nh.param<std::string>("/leftCameraTopic", "/camera_back/image_raw");
    rightCameraSub.subscribe(nh, rightCameraTopic, qSize);
    leftCameraSub.subscribe(nh, leftCameraTopic, qSize);

    ////Right and left camera synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> camerasSyncPolicy;
    message_filters::Synchronizer<camerasSyncPolicy> camerasSynchronizer(camerasSyncPolicy(qSize), rightCameraSub, leftCameraSub, cameraPoseSub);
    camerasSynchronizer.registerCallback(boost::bind(&dualFisheyeVS::camerasImageCallback,this, _1, _2, _3));

    ////Image displayers
    rightI.resize(rightCameraParam.height, rightCameraParam.width);
    leftI.resize(rightCameraParam.height, rightCameraParam.width);
    rightId.resize(rightCameraParam.height, rightCameraParam.width);
    leftId.resize(rightCameraParam.height, rightCameraParam.width);
    rightIdiff.resize(rightCameraParam.height, rightCameraParam.width);
    leftIdiff.resize(rightCameraParam.height, rightCameraParam.width);
    leftDispI.init(leftI, 0, 27, "LEFT I");
    leftDispId.init(leftId, 0, 27+37+rightCameraParam.height, "LEFT I*");
    leftDispIdiff.init(leftIdiff, 0,  27+(37+rightCameraParam.height)*2, "LEFT DIFF");
    rightDispI.init(rightI, rightCameraParam.width, 27, "RIGHT I");
    rightDispId.init(rightId, rightCameraParam.width,  27+37+rightCameraParam.height, "RIGHT I*");
    rightDispIdiff.init(rightIdiff, rightCameraParam.width,  27+(37+rightCameraParam.height)*2, "RIGHT DIFF");

    ////Plots
    plot.init(4,rightCameraParam.height*3,rightCameraParam.width*2,rightCameraParam.width*2,0);
    plot.initGraph(0,1); plot.setTitle(0,"Photometric error"); plot.setLegend(0,0,"error");
    plot.initGraph(1,6); plot.setTitle(1,"Velocities");
    plot.setLegend(1,0,"v_x");     plot.setLegend(1,1,"v_y");     plot.setLegend(1,2,"v_z");
    plot.setLegend(1,3,"w_x");     plot.setLegend(1,4,"w_y");     plot.setLegend(1,5,"w_z");
    plot.initGraph(2,3); plot.setTitle(2,"Translation error");
    plot.setLegend(2,0,"dtx_x");     plot.setLegend(2,1,"dt_y");     plot.setLegend(2,2,"dt_z");
    plot.initGraph(3,3); plot.setTitle(3,"Orientation error");
    plot.setLegend(3,0,"dw_x");     plot.setLegend(3,1,"dw_y");     plot.setLegend(3,2,"dw_z");


    ////Camera desired and initial pose initialization
    cameraPosesInitialization();

    ////Start VS Node
    ros::spin();
}


void dualFisheyeVS::cameraPosesInitialization(){
    std::cout<<"Go to the desired pose then click on LEFT I"<<std::endl;
    do{
        leftId = leftI;
        rightId = rightI;
        vpDisplay::display(leftId);
        vpDisplay::display(rightId);
        vpDisplay::flush(leftId);
        vpDisplay::flush(rightId);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    cdpo.buildFrom(visp_bridge::toVispHomogeneousMatrix((*ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/camera_link_controller/pose")).pose).inverse());

    std::cout<<"Go to the initial pose then click on LEFT I"<<std::endl;
    do{
        vpImageTools::imageDifference(leftI, leftId, leftIdiff);
        vpImageTools::imageDifference(rightI, rightId, rightIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));

    ////Build desired luminance feature
    buildLuminanceFeature(leftId, leftCameraParam, leftSd, false);
    buildLuminanceFeature(rightId, rightCameraParam, rightSd, false );
    sd.clear();
    sd.insert(sd.begin(), rightSd.begin(), rightSd.end());
    sd.insert(sd.end(), leftSd.begin(), leftSd.end());

    iter = 0;
    vsStarted = true;
}


void dualFisheyeVS::camerasImageCallback(const sensor_msgs::Image::ConstPtr& frontImage, const sensor_msgs::Image::ConstPtr& backImage, const geometry_msgs::PoseStamped::ConstPtr& cameraPose){
    ////Real acquired images are the desired ones
    rightI = visp_bridge::toVispImage(*frontImage);
    leftI = visp_bridge::toVispImage(*backImage);
    ////Get the current camera pose
    cMo = visp_bridge::toVispHomogeneousMatrix((*cameraPose).pose).inverse();
    cpo.buildFrom(cMo);

    if(vsStarted){
        ////Plots
        for(int i=0;i<3;i++)
            plot.plot(2,i,iter,sqrt(cpo[i]*cpo[i]-cdpo[i]*cdpo[i]));

        for(int i=0;i<3;i++)
            plot.plot(3,i,iter,vpMath::deg(sqrt(cpo[i+3]*cpo[i+3]-cdpo[i+3]*cdpo[i+3])));

        ////Displays
        vpImageTools::imageDifference(rightI, rightId,rightIdiff);
        vpImageTools::imageDifference(leftI, leftId,leftIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);

        ////Build current luminance feature (virtual images as binary masks)
        leftS.clear(); rightS.clear(); s.clear();
        buildLuminanceFeature(leftI, leftCameraParam, leftS, true);
        buildLuminanceFeature(rightI, rightCameraParam, rightS, true);
        s.insert(s.begin(),rightS.begin(),rightS.end());
        s.insert(s.end(),leftS.begin(),leftS.end());

        ////Compute interaction matrix
        cLr.clear(); cLl.clear(); L.clear();
        computeLuminanceInteractionMatrixOmni(rightS, cLr,rightCameraParam, rMc);
        computeLuminanceInteractionMatrixOmni(leftS, cLl, leftCameraParam, lMc);
        L.stack(cLr);
        L.stack(cLl);

        ////Compute error vector
        computeErrorVector(s, sd, e);

        ////Control law (Gauss-Newton)
        v.resize(6);
        v = -gain*L.pseudoInverse()*e;
        cameraPosePub.publish(visp_bridge::toGeometryMsgsPose(cMo.inverse() * vpExponentialMap::direct(v,1.0) ));

        ////Plots
        plot.plot(0,0,iter,e.sumSquare());
        for(int i=0;i<6;i++)
            plot.plot(1,i,iter,v[i]);


        iter++;
    }
}

cameraParameters dualFisheyeVS::cameraInfoToCameraParam(sensor_msgs::CameraInfo msg){
    cameraParameters cam;
    cam.height = msg.height;
    cam.width = msg.width;
    cam.au = msg.K[0];
    cam.av = msg.K[4];
    cam.u0 = msg.K[2];
    cam.v0 = msg.K[5];
    cam.xi = msg.D[0];

    return cam;
}

vpHomogeneousMatrix dualFisheyeVS::getvpMatrixfromTransform(std::string frame_i, std::string frame_o){
    vpHomogeneousMatrix oMi(0,0,0,0,0,0);
    tf::StampedTransform oMi_tf;
    tf::TransformListener listener;
    listener.waitForTransform(frame_o, frame_i, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(frame_o, frame_i, ros::Time(0), oMi_tf);
    oMi.buildFrom(vpTranslationVector(oMi_tf.getOrigin().x(), oMi_tf.getOrigin().y(), oMi_tf.getOrigin().z()), vpQuaternionVector(oMi_tf.getRotation().x(), oMi_tf.getRotation().y(), oMi_tf.getRotation().z(),oMi_tf.getRotation().w() ) );
    return oMi;
}

geometry_msgs::Pose dualFisheyeVS::getGeoPoseFromTransform(std::string frame_i, std::string frame_o){
    geometry_msgs::Pose oMi;
    tf::StampedTransform oMi_tf;
    tf::TransformListener listener;
    listener.waitForTransform(frame_o, frame_i, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(frame_o, frame_i, ros::Time(0), oMi_tf);

    oMi.position.x = oMi_tf.getOrigin().x();
    oMi.position.y = oMi_tf.getOrigin().y();
    oMi.position.z = oMi_tf.getOrigin().z();
    oMi.orientation.x = oMi_tf.getRotation().x();
    oMi.orientation.y = oMi_tf.getRotation().y();
    oMi.orientation.z = oMi_tf.getRotation().z();
    oMi.orientation.w = oMi_tf.getRotation().w();

    return oMi;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dualFisheyeVS");
    dualFisheyeVS(argc, argv);
}