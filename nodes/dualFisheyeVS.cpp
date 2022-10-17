/*
 * Dual Fisheye Visual Servoing
 * Author: Nathan Crombez (CIAD-UTBM)
 * Date: September 2022
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
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/GetCurrentPose.h>

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


#include "../include/dualFisheyeVS/dualFisheyeVS.h"

class dualFisheyeVS{
private:
    ros::NodeHandle nh;
    int qSize;
    message_filters::Subscriber<sensor_msgs::Image> rightCameraSub, leftCameraSub, externalCameraSub;

    ros::ServiceClient armMoveLine;
    dsr_msgs::MoveLine moveLineMsg;

    ros::ServiceClient getCurrentArmPose;
    dsr_msgs::GetCurrentPose currentArmPoseMsg;

    std::string rightCameraTopic, leftCameraTopic, externalCameraTopic;
    std::string rightCameraInfoTopic, leftCameraInfoTopic;

    vpImage<unsigned char> rightI, leftI, rightId, leftId, rightIdiff, leftIdiff;
    cameraParameters rightCameraParam, leftCameraParam;

    std::vector<luminanceFeature> rightS, rightSd, leftS, leftSd, s, sd;
    vpHomogeneousMatrix rMf, lMf;
    vpPoseVector fdpo, fpo;

    vpMatrix L, fLr, fLl;
    vpColVector e, v;
    float gain;
    int iter;

    vpDisplayX rightDispI, leftDispI, rightDispId, leftDispId, rightDispIdiff, leftDispIdiff;

    vpPlot plot;

    bool vsStarted, saveData, saveExternalViews;

    std::string dataFolderName;
    std::ofstream ficData;



public:
    dualFisheyeVS(int argc, char **argv);

    void camerasImageCallback2(const sensor_msgs::Image::ConstPtr& rightImage, const sensor_msgs::Image::ConstPtr& leftImage, const sensor_msgs::Image::ConstPtr& externalImage);
    void camerasImageCallback(const sensor_msgs::Image::ConstPtr& rightImage, const sensor_msgs::Image::ConstPtr& leftImage);
    void cameraPosesInitialization();
    cameraParameters cameraInfoToCameraParam(sensor_msgs::CameraInfo msg);
    geometry_msgs::Pose getGeoPoseFromTransform(std::string frame_i, std::string frame_o);
    vpHomogeneousMatrix getvpMatrixfromTransform(std::string frame_i, std::string frame_o);
    vpHomogeneousMatrix vpMatrixfromDoosanPose(double p[6]);
    vpColVector poseDifference(vpHomogeneousMatrix cMo, vpHomogeneousMatrix cdMo);


    };


dualFisheyeVS::dualFisheyeVS(int argc, char **argv) : nh(){
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    qSize = 1;
    vsStarted = false;
    v.resize(6);

    ////Get parameters
    saveData = nh.param("/dualFisheyeVS/saveData", 0);
    saveExternalViews = nh.param("/dualFisheyeVS/saveExternalViews", 0);
    gain = nh.param("/dualFisheyeVS/gain", 1.0);
    rightCameraInfoTopic = nh.param<std::string>("/dualFisheyeVS/rightCameraInfoTopic", "/insta360/right/camera_info");
    leftCameraInfoTopic = nh.param<std::string>("/dualFisheyeVS/leftCameraInfoTopic", "/insta360/left/camera_info");
    rightCameraTopic = nh.param<std::string>("/dualFisheyeVS/rightCameraTopic", "/insta360/right/image_raw");
    leftCameraTopic = nh.param<std::string>("/dualFisheyeVS/leftCameraTopic", "/insta360/left/image_raw");
    externalCameraTopic = nh.param<std::string>("/dualFisheyeVS/externalCameraTopic", "/camera/image_raw");


    ////Name of the experiment (date+time)
    boost::posix_time::ptime now = boost::posix_time::second_clock::universal_time();
    dataFolderName = boost::lexical_cast<std::string>(now.date())+""+boost::lexical_cast<std::string>(now.time_of_day());
    boost::erase_all(dataFolderName, "-");
    boost::erase_all(dataFolderName, ":");

    if(saveData) {
        boost::filesystem::create_directory(dataFolderName);
        ficData.open(dataFolderName + "/data.dat");
        if(saveExternalViews){
            boost::filesystem::create_directory( dataFolderName + "/Views");
            boost::filesystem::create_directory( dataFolderName + "/Views/Desired");
        }
        boost::filesystem::create_directory( dataFolderName + "/I");
        boost::filesystem::create_directory( dataFolderName + "/I/R");
        boost::filesystem::create_directory( dataFolderName + "/I/L");
        boost::filesystem::create_directory( dataFolderName + "/Id");
        boost::filesystem::create_directory( dataFolderName + "/Id/R");
        boost::filesystem::create_directory( dataFolderName + "/Id/L");
        boost::filesystem::create_directory( dataFolderName + "/Idiff");
        boost::filesystem::create_directory( dataFolderName + "/Idiff/R");
        boost::filesystem::create_directory( dataFolderName + "/Idiff/L");
    }

    ////Get right and left camera intrinsic parameters
    rightCameraParam = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightCameraInfoTopic, nh));
    leftCameraParam = cameraInfoToCameraParam(*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(leftCameraInfoTopic, nh));

    ////Get right and left camera extrinsic parameters
    rMf = getvpMatrixfromTransform("/link6", "/camera_right_optical_frame");
    lMf = getvpMatrixfromTransform("/link6", "/camera_left_optical_frame");

    ////Robot pose controler
    armMoveLine = nh.serviceClient<dsr_msgs::MoveLine>("/dsr01a0509/motion/move_line");
    getCurrentArmPose = nh.serviceClient<dsr_msgs::GetCurrentPose>("/dsr01a0509/system/get_current_pose");
    currentArmPoseMsg.request.space_type = 1; //space pose requested

    ////Right and left camera subscribers
    rightCameraSub.subscribe(nh, rightCameraTopic, qSize);
    leftCameraSub.subscribe(nh, leftCameraTopic, qSize);

    ////Right and left (and external ) cameras synchronizer
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>> camerasSynchronizer2(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>(qSize), rightCameraSub, leftCameraSub, externalCameraSub);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>> camerasSynchronizer(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(qSize), rightCameraSub, leftCameraSub);
    if(saveExternalViews){
        externalCameraSub.subscribe(nh, externalCameraTopic, qSize);
        camerasSynchronizer2.registerCallback(boost::bind(&dualFisheyeVS::camerasImageCallback2,this, _1, _2, _3));
    }else{
        camerasSynchronizer.registerCallback(boost::bind(&dualFisheyeVS::camerasImageCallback,this, _1, _2));
    }

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
    plot.init(4,(37+rightCameraParam.height)*3-37,rightCameraParam.width*3,rightCameraParam.width*2,27);
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
    ROS_WARN("Go to the desired pose then click on LEFT I");
    do{
        leftId = leftI;
        rightId = rightI;
        vpDisplay::display(leftId);
        vpDisplay::display(rightId);
        vpDisplay::flush(leftId);
        vpDisplay::flush(rightId);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    getCurrentArmPose.call(currentArmPoseMsg);
    fdpo.buildFrom(vpMatrixfromDoosanPose(currentArmPoseMsg.response.pos.elems));
    ROS_DEBUG("Desired Pose : %f %f %f %f %f %f", fdpo[0], fdpo[1], fdpo[2], fdpo[3], fdpo[4], fdpo[5]);
    if(saveData && saveExternalViews){
        vpImage<vpRGBa> externalView;
        visp_bridge::toVispImageRGBa(*ros::topic::waitForMessage<sensor_msgs::Image>(externalCameraTopic, nh)).quarterSizeImage(externalView);
        vpImageIo::write(externalView, dataFolderName + "/Views/Desired/View.png");
    }

    ROS_WARN("Go to the initial pose then click on LEFT I");
    do{
        vpImageTools::imageDifference(leftI, leftId, leftIdiff);
        vpImageTools::imageDifference(rightI, rightId, rightIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);
        ros::spinOnce();
    }while(!vpDisplay::getClick(leftI, false));
    fpo.buildFrom(vpMatrixfromDoosanPose(currentArmPoseMsg.response.pos.elems));
    ROS_DEBUG("Initial Pose : %f %f %f %f %f %f", fpo[0], fpo[1], fpo[2], fpo[3], fpo[4], fpo[5]);

    ////Save experiment data
    if(saveData) {
        ficData<<"#########"<<std::endl;
        ficData<<"gain "<<std::endl;
        ficData<<gain<<std::endl;
        ficData<<"#########"<<std::endl;
        ficData<<"RIGHT CAMERA"<<std::endl;
        ficData<<"rMf "<<std::endl;
        ficData << vpPoseVector(rMf).t() << std::endl;
        ficData<<"width height au av u0 v0 xi"<<std::endl;
        ficData<<rightCameraParam.width<<" "<<rightCameraParam.height<<" "<<rightCameraParam.au<<" "<<rightCameraParam.av<<" "<<rightCameraParam.u0<<" "<<rightCameraParam.v0<<std::endl;
        ficData<<"LEFT CAMERA"<<std::endl;
        ficData<<"lMf "<<std::endl;
        ficData << vpPoseVector(lMf).t() << std::endl;
        ficData<<"width height au av u0 v0 xi"<<std::endl;
        ficData<<leftCameraParam.width<<" "<<leftCameraParam.height<<" "<<leftCameraParam.au<<" "<<leftCameraParam.av<<" "<<leftCameraParam.u0<<" "<<leftCameraParam.v0<<std::endl;
        ficData<<"#########"<<std::endl;
        ficData<<"DESIRED FLANGE POSE"<<std::endl;
        ficData<<"pxd pyd pzd rxd ryd rzd"<<std::endl;
        ficData<<vpHomogeneousMatrix(fdpo).inverse().getTranslationVector().t()<<" "<<vpRxyzVector(fdpo.getRotationMatrix()).t()<<std::endl;
        ficData<<"#########"<<std::endl;
        ficData<<"INITIAL FLANGE POSE"<<std::endl;
        ficData<<"pxi pyi pzi rxi ryi rzi"<<std::endl;
        ficData<<vpHomogeneousMatrix(fpo).inverse().getTranslationVector().t()<<" "<<vpRxyzVector(fpo.getRotationMatrix()).t()<<std::endl;
        ficData<<"#########"<<std::endl;
        ficData<<"CURRENT FLANGE POSE"<<std::endl;
        ficData<<"iteration px py pz rx ry rz vx vy vz wx wy wz epx epy epz erx ery erz error"<<std::endl;

        vpImageIo::write(rightId, dataFolderName + "/Id/R/Id.png");
        vpImageIo::write(leftId, dataFolderName + "/Id/L/Id.png");
    }

    ////Build desired luminance feature
    buildLuminanceFeature(leftId, leftCameraParam, leftSd, false);
    buildLuminanceFeature(rightId, rightCameraParam, rightSd, false );
    sd.clear();
    sd.insert(sd.begin(), rightSd.begin(), rightSd.end());
    sd.insert(sd.end(), leftSd.begin(), leftSd.end());

    iter = 0;
    vsStarted = true;
}


void dualFisheyeVS::camerasImageCallback(const sensor_msgs::Image::ConstPtr& rightImage, const sensor_msgs::Image::ConstPtr& leftImage){
    ////Real acquired images are the desired ones
     visp_bridge::toVispImage(*rightImage).quarterSizeImage(rightI);
     visp_bridge::toVispImage(*leftImage).quarterSizeImage(leftI);
    ////Get current arm pose
    getCurrentArmPose.call(currentArmPoseMsg);
    fpo.buildFrom(vpMatrixfromDoosanPose(currentArmPoseMsg.response.pos.elems));

    if(vsStarted){
        vpColVector poseError = poseDifference(vpHomogeneousMatrix(fpo), vpHomogeneousMatrix(fdpo));

        ////Plots
        for(int i=0;i<3;i++)
            plot.plot(2,i,iter,poseError[i]);

        for(int i=0;i<3;i++) {
            plot.plot(3, i, iter, vpMath::deg(poseError[i+3]));
        }

        ////Displays
        vpImageTools::imageDifference(rightI, rightId,rightIdiff);
        vpImageTools::imageDifference(leftI, leftId,leftIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);

        ////Save Images
        if(saveData){
            std::ostringstream ss;
            ss << std::setw(4) << std::setfill('0') << iter;
            vpImageIo::write(rightI, dataFolderName + "/I/R/I_"+ss.str()+".png");
            vpImageIo::write(leftI, dataFolderName + "/I/L/I_"+ss.str()+".png");
            vpImageIo::write(rightIdiff, dataFolderName + "/Idiff/R/Idiff_"+ss.str()+".png");
            vpImageIo::write(leftIdiff, dataFolderName + "/Idiff/L/Idiff_"+ss.str()+".png");
        }

        ////Build current luminance feature (virtual images as binary masks)
        leftS.clear(); rightS.clear(); s.clear();
        buildLuminanceFeature(leftI, leftCameraParam, leftS, true);
        buildLuminanceFeature(rightI, rightCameraParam, rightS, true);
        s.insert(s.begin(),rightS.begin(),rightS.end());
        s.insert(s.end(),leftS.begin(),leftS.end());

        ////Compute interaction matrix
        fLr.clear(); fLl.clear(); L.clear();
        computeLuminanceInteractionMatrixOmni(rightS, fLr, rightCameraParam, rMf);
        computeLuminanceInteractionMatrixOmni(leftS, fLl, leftCameraParam, lMf);
        L.stack(fLr);
        L.stack(fLl);

        ////Compute error vector
        computeErrorVector(s, sd, e);

        ////Control law (Gauss-Newton)
        v = -gain*L.pseudoInverse()*e;
        vpHomogeneousMatrix T(vpExponentialMap::direct(v,0.3)); // ^link6M_newlink6
        vpTranslationVector t = T.getTranslationVector();
        vpRzyzVector Rzyz(T.getRotationMatrix());

        moveLineMsg.request.ref = 1; //tool ref
        moveLineMsg.request.mode = 1; //relative pose
        moveLineMsg.request.time = 0.3; //time to reach the new pose (if time is set vel are ignored)
        moveLineMsg.request.blendType = 1; //BLENDING_SPEED_TYPE_OVERRIDE
        //moveLineMsg.request.vel = {sqrt(t.sumSquare())*1000,0}; //linear velocities (angular vel is determined prop to the linear one)
        moveLineMsg.request.pos = {t[0]*1000, t[1]*1000, t[2]*1000, vpMath::deg(Rzyz[0]), vpMath::deg(Rzyz[1]), vpMath::deg(Rzyz[2])};    //m,m,m,deg,deg,deg
        ROS_DEBUG("Displacement : %f %f %f %f %f %f",moveLineMsg.request.pos.elems[0],moveLineMsg.request.pos.elems[1],moveLineMsg.request.pos.elems[2],moveLineMsg.request.pos.elems[3],moveLineMsg.request.pos.elems[4],moveLineMsg.request.pos.elems[5]);
        ROS_DEBUG("Current Pose : %f %f %f %f %f %f", fpo[0], fpo[1], fpo[2], fpo[3], fpo[4], fpo[5]);
        ROS_DEBUG("Desired Pose : %f %f %f %f %f %f", fdpo[0], fdpo[1], fdpo[2], fdpo[3], fdpo[4], fdpo[5]);
        ROS_DEBUG("Pose error : %f %f %f %f %f %f",poseError[0], poseError[1], poseError[2], poseError[3], poseError[4], poseError[5]);
        armMoveLine.call(moveLineMsg);

        ////Save data
        if(saveData){
            ficData << iter << " " << vpHomogeneousMatrix(fpo).inverse().getTranslationVector().t()<<" "<<vpRxyzVector(fpo.getRotationMatrix()).t()
                    << " " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5]
                    << " " << poseError[0] << " " << poseError[1] << " " << poseError[2] << " " << poseError[3] << " " << poseError[4] << " " << poseError[5] <<" "<< e.sumSquare() << std::endl;
        }


        ////Plots
        plot.plot(0,0,iter,e.sumSquare());
        for(int i=0;i<6;i++)
            plot.plot(1,i,iter,v[i]);

        iter++;
    }
}


void dualFisheyeVS::camerasImageCallback2(const sensor_msgs::Image::ConstPtr& rightImage, const sensor_msgs::Image::ConstPtr& leftImage, const sensor_msgs::Image::ConstPtr& externalImage){
    ////Real acquired images are the desired ones
    visp_bridge::toVispImage(*rightImage).quarterSizeImage(rightI);
    visp_bridge::toVispImage(*leftImage).quarterSizeImage(leftI);
    ////Get current pose
    getCurrentArmPose.call(currentArmPoseMsg);
    fpo.buildFrom(vpMatrixfromDoosanPose(currentArmPoseMsg.response.pos.elems));

    if(vsStarted){

        vpColVector poseError = poseDifference(vpHomogeneousMatrix(fpo), vpHomogeneousMatrix(fdpo));

        ////Plots
        for(int i=0;i<3;i++)
            plot.plot(2,i,iter,poseError[i]);

        for(int i=0;i<3;i++) {
            plot.plot(3, i, iter, vpMath::deg(poseError[i+3]));
        }

        ////Displays
        vpImageTools::imageDifference(rightI, rightId,rightIdiff);
        vpImageTools::imageDifference(leftI, leftId,leftIdiff);
        vpDisplay::display(leftI); vpDisplay::flush(leftI);
        vpDisplay::display(rightI); vpDisplay::flush(rightI);
        vpDisplay::display(leftIdiff); vpDisplay::flush(leftIdiff);
        vpDisplay::display(rightIdiff); vpDisplay::flush(rightIdiff);

        ////Save Images
        if(saveData){
            std::ostringstream ss;
            ss << std::setw(4) << std::setfill('0') << iter;
            vpImageIo::write(rightI, dataFolderName + "/I/R/I_"+ss.str()+".png");
            vpImageIo::write(leftI, dataFolderName + "/I/L/I_"+ss.str()+".png");
            vpImageIo::write(rightIdiff, dataFolderName + "/Idiff/R/Idiff_"+ss.str()+".png");
            vpImageIo::write(leftIdiff, dataFolderName + "/Idiff/L/Idiff_"+ss.str()+".png");
            vpImage<vpRGBa> externalView;
            visp_bridge::toVispImageRGBa(*externalImage).quarterSizeImage(externalView);
            vpImageIo::write(externalView, dataFolderName + "/Views/View_"+ss.str()+".png");
        }

        ////Build current luminance feature (virtual images as binary masks)
        leftS.clear(); rightS.clear(); s.clear();
        buildLuminanceFeature(leftI, leftCameraParam, leftS, true);
        buildLuminanceFeature(rightI, rightCameraParam, rightS, true);
        s.insert(s.begin(),rightS.begin(),rightS.end());
        s.insert(s.end(),leftS.begin(),leftS.end());

        ////Compute interaction matrix
        fLr.clear(); fLl.clear(); L.clear();
        computeLuminanceInteractionMatrixOmni(rightS, fLr, rightCameraParam, rMf);
        computeLuminanceInteractionMatrixOmni(leftS, fLl, leftCameraParam, lMf);
        L.stack(fLr);
        L.stack(fLl);

        ////Compute error vector
        computeErrorVector(s, sd, e);

        ////Control law (Gauss-Newton)
        v = -gain*L.pseudoInverse()*e;
        vpHomogeneousMatrix T(vpExponentialMap::direct(v,0.3)); // ^link6M_newlink6
        vpTranslationVector t = T.getTranslationVector();
        vpRzyzVector Rzyz(T.getRotationMatrix());

        moveLineMsg.request.ref = 1; //tool ref
        moveLineMsg.request.mode = 1; //relative pose
        moveLineMsg.request.time = 0.3; //time to reach the new pose (if time is set vel are ignored)
        moveLineMsg.request.blendType = 1; //BLENDING_SPEED_TYPE_OVERRIDE
        //moveLineMsg.request.vel = {sqrt(t.sumSquare())*1000,0}; //linear velocities (angular vel is determined prop to the linear one)
        moveLineMsg.request.pos = {t[0]*1000, t[1]*1000, t[2]*1000, vpMath::deg(Rzyz[0]), vpMath::deg(Rzyz[1]), vpMath::deg(Rzyz[2])};    //m,m,m,deg,deg,deg
        ROS_DEBUG("Displacement : %f %f %f %f %f %f",moveLineMsg.request.pos.elems[0],moveLineMsg.request.pos.elems[1],moveLineMsg.request.pos.elems[2],moveLineMsg.request.pos.elems[3],moveLineMsg.request.pos.elems[4],moveLineMsg.request.pos.elems[5]);
        ROS_DEBUG("Current Pose : %f %f %f %f %f %f", fpo[0], fpo[1], fpo[2], fpo[3], fpo[4], fpo[5]);
        ROS_DEBUG("Desired Pose : %f %f %f %f %f %f", fdpo[0], fdpo[1], fdpo[2], fdpo[3], fdpo[4], fdpo[5]);
        ROS_DEBUG("Pose error : %f %f %f %f %f %f",poseError[0], poseError[1], poseError[2], poseError[3], poseError[4], poseError[5]);
        armMoveLine.call(moveLineMsg);

        ////Save data
        if(saveData){
            ficData << iter << " " << vpHomogeneousMatrix(fpo).inverse().getTranslationVector().t()<<" "<<vpRxyzVector(fpo.getRotationMatrix()).t()
                    << " " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5]
                    << " " << poseError[0] << " " << poseError[1] << " " << poseError[2] << " " << poseError[3] << " " << poseError[4] << " " << poseError[5] <<" "<< e.sumSquare() << std::endl;
        }


        ////Plots
        plot.plot(0,0,iter,e.sumSquare());
        for(int i=0;i<6;i++)
            plot.plot(1,i,iter,v[i]);


        iter++;
    }
}



cameraParameters dualFisheyeVS::cameraInfoToCameraParam(sensor_msgs::CameraInfo msg){
    cameraParameters cam;
    cam.height = msg.height/4;
    cam.width = msg.width/4;
    cam.au = msg.K[0]/4.0;
    cam.av = msg.K[4]/4.0;
    cam.u0 = msg.K[2]/4.0;
    cam.v0 = msg.K[5]/4.0;
    cam.xi = msg.D[4];

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

vpHomogeneousMatrix dualFisheyeVS::vpMatrixfromDoosanPose(double p[6]){
    vpHomogeneousMatrix M;
    vpTranslationVector t(p[0],p[1],p[2]);
    vpRzyzVector R(vpMath::rad(p[3]), vpMath::rad(p[4]), vpMath::rad(p[5]));
    M.buildFrom(t, vpRotationMatrix(R));
    return M.inverse();
}

vpColVector dualFisheyeVS::poseDifference(vpHomogeneousMatrix cMo, vpHomogeneousMatrix cdMo){
    vpColVector diff(6);
    vpHomogeneousMatrix cMcd = cMo * cdMo.inverse();
    //Erreur en position
    vpTranslationVector cOcd = cMcd.getTranslationVector();

    //Erreur en rotation
    vpColVector cXcd = cMcd.getCol(0);
    vpColVector cYcd = cMcd.getCol(1);
    vpColVector cZcd = cMcd.getCol(2);

    diff[0] = cOcd[0];
    diff[1] = cOcd[1];
    diff[2] = cOcd[2];
    diff[3] = acos(cXcd[0]);
    diff[4] = acos(cYcd[1]);
    diff[5] = acos(cZcd[2]);

    return diff;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dualFisheyeVS");
    dualFisheyeVS(argc, argv);
}