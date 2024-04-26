/**
* @file     affine_trans.cpp
* @brief    アフィン変換実行クラスAffineTransのソースファイル
* @author   S.Kumada
* @date     2023/06/18
* @note     アフィン変換により、地図画像の平行移動・回転処理を行うクラスの実装
*/

#include "affine_trans_of_map/affine_trans.hpp"
#include "utilitie.h"


void AffineTrans::DebugImgShow(std::string msg)
{
    std::string show_msg = msg + "_image";
    cv::imshow("Img",output_img_);
    cv::waitKey(0);
}

AffineTrans::AffineTrans()
{
    // 初期化処理
    ros::NodeHandle param_nh("~");

    // パラメータ初期化
    intensity_ = 205;

}

AffineTrans::~AffineTrans()
{
    // 何もしない

#ifdef DEBUG
    cv::destroyAllWindows();
#endif
}

void AffineTrans::setSourceImg(nav_msgs::OccupancyGrid map_data)
{
    this->input_map_ = map_data;
    this->source_img_ = mapToMat(map_data);
}

void AffineTrans::setSourceImg(cv::Mat source_img)
{
    this->source_img_ = source_img;
}

void AffineTrans::setRotationCenter(int coord_x, int coord_y)
{
    this->rotation_center_.x = coord_x;
    this->rotation_center_.y = coord_y;
}

void AffineTrans::setShiftAmount(double x_axis, double y_axis)
{
    this->x_axis_ = x_axis;
    this->y_axis_ = y_axis;
}

void AffineTrans::setRotationYaw(double rotation_yaw)
{
    this->rotation_angle_ = rotation_yaw;
}

void AffineTrans::setConvertedImgSize(unsigned int width, unsigned int height)
{
    this->width_ = width;
    this->height_ = height;
}

void AffineTrans::setConvertedOriginCoord(geometry_msgs::Pose origin)
{
    this->converted_map_origin_ = origin;
}

void AffineTrans::setTranceformMode(tranceform_mode_t mode)
{
    this->tranceform_mode_ = mode;
}

void AffineTrans::setMapFrame(std::string map_frame)
{
    this->map_frame_ = map_frame;
}

void AffineTrans::setIntensity(int intensity)
{
    this->intensity_ = intensity;
}

nav_msgs::OccupancyGrid AffineTrans::getResultImg(void)
{
    return matToMap(this->output_img_);
}

cv::Mat AffineTrans::mapToMat(nav_msgs::OccupancyGrid grid_map)
{
    double yaw, pitch, roll, map_theta; // オイラー角度計算用
    cv::Mat img_out = cv::Mat::zeros(cv::Size(grid_map.info.width, grid_map.info.height), CV_8UC1); // 画像データに変換した地図
    
    // 姿勢情報のQuaternion⇒オイラー角を計算する
    geometry_msgs::Quaternion orientation = grid_map.info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    mat.getEulerYPR(yaw, pitch, roll);
    map_theta = yaw;

#ifdef DEBUG
    ROS_INFO("DEBUG : Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
    grid_map.info.width,grid_map.info.height,grid_map.info.resolution,
    grid_map.info.origin.position.x,grid_map.info.origin.position.y,map_theta);
#endif

    // 変換処理（不明領域-1が考慮されていない気がするが、現状-1/0/100の値しか取りえないため、問題にはならない）
    for (unsigned int y = 0; y < grid_map.info.height; y++)
    { // 高さ分ループ（1行分の処理）
        for (unsigned int x = 0; x < grid_map.info.width; x++)
        { // 幅分ループ（1行分の各要素に対する処理）
            unsigned int i = x + (grid_map.info.height - y - 1) * grid_map.info.width;
            int intensity = intensity_; // 不明領域埋め用
            if(grid_map.data[i] != -1 && grid_map.data[i] != 0 && grid_map.data[i] != 100)
            {
                std::cout << grid_map.data[i] << " / ";
            }

            if (grid_map.data[i] >= 0 && grid_map.data[i] <= 100)
            { // 値のスケーリング（0~100 0⇒ 0～255）
                intensity = round((float)(100.0 - grid_map.data[i]) * 2.55);
            }
            img_out.at<unsigned char>(y, x) = intensity;
        }
    }

#ifdef DEBUG // 確認用
    // cv::imshow("GridMap TO Mat",img_out);
    // cv::waitKey(0);
#endif

    return img_out;
}


nav_msgs::OccupancyGrid AffineTrans::matToMap(cv::Mat mat_map)
{
    nav_msgs::OccupancyGrid grid_map;
    grid_map.header.frame_id = map_frame_;
    grid_map.header.stamp = ros::Time::now();
    grid_map.info.map_load_time = input_map_.info.map_load_time;
    grid_map.info.resolution = input_map_.info.resolution;
    grid_map.info.width = mat_map.cols;
    grid_map.info.height = mat_map.rows;
    grid_map.info.origin = converted_map_origin_; // 原点を中心の回転画像のため

#if DEBUG
    ROS_INFO("DEBUG : Converted map info");
    std::cout << "resolution: " << grid_map.info.resolution  << std::endl;
    std::cout << "width: " << grid_map.info.width  << std::endl;
    std::cout << "height: " << grid_map.info.height  << std::endl;
    std::cout << "origin: " << grid_map.info.origin  << std::endl;
#endif
    
    grid_map.data.resize(grid_map.info.width * grid_map.info.height);

    // data変換
    for (unsigned int y = 0; y < grid_map.info.height; y++)
    { // 高さ分ループ（1行分の処理）
        for (unsigned int x = 0; x < grid_map.info.width; x++)
        { // 幅分ループ（1行分の各要素に対する処理）
            // gridmap インデックス計算
            unsigned int i = x + (grid_map.info.height - y - 1) * grid_map.info.width;
            if(output_img_.at<unsigned char>(y, x) == intensity_)
            {
                grid_map.data[i] = -1;
            }
            else if(output_img_.at<unsigned char>(y, x) == 0)
            {
                grid_map.data[i] = 100;
            }
            else if(output_img_.at<unsigned char>(y, x) == 255)
            {
                grid_map.data[i] = 0;
            }
            else
            {
#ifdef DEBUG
                std::cout << (int)output_img_.at<unsigned char>(y, x) << " / ";
#endif
            }
        }
    }

    return (grid_map);
}

void AffineTrans::shiftConvert()
{
#ifdef DEBUG
    // 変換前
    // cv::imshow("Img",source_img_);
    // cv::waitKey(0);
#endif
    // 変換行列を求める
    cv::Mat shift_matrix = calcShiftMatrix(x_axis_, y_axis_);

    // 変換処理実行
    affinConversion(shift_matrix, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(intensity_, 0, 0));
}

void AffineTrans::rotateConvert()
{
    // 回転軸の中心座標
    cv::Point2f rotation_center(rotation_center_.x, rotation_center_.y);
    
    // 変換行列を求める
    cv::Mat rotate_matrix = calcRotateMatrix(rotation_center, rotation_angle_, 1);

    // 変換処理実行
    affinConversion(rotate_matrix, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(intensity_, 0, 0));
}

void AffineTrans::affinConversion(cv::Mat matrix, int interepolation, int borderMode, const cv::Scalar borderValue)
{
    // assart
    if(source_img_.empty())
    { // 変換対象のデータがない場合
        ROS_ERROR("Data for the map to be converted does not exist.");
        exit(1);
    }

    // 変換後の画像サイズ
    cv::Size img_size(width_, height_);

    // 変換方向
    if(tranceform_mode_ == TRANCEFORM_MODE_INVERSE)
    { // 逆方向変換の場合

#if DEBUG
        ROS_INFO("DEBUG : Setting tranceform mode inverse");
#endif
        // 変換行列の逆行列を求める
        cv::Mat inverse_matrix;
        cv::invertAffineTransform(matrix, inverse_matrix);
        matrix = inverse_matrix;
    }

#if DEBUG
    ROS_INFO("DEBUG : Affine conversion info");
    std::cout << "matrix: " << "\n" << matrix << std::endl;
    std::cout << "img size: " << img_size << std::endl;
    std::cout << "interepolation: " << interepolation << std::endl;
    std::cout << "border mode: " << borderMode << std::endl;
    std::cout << "border value: " << borderValue << std::endl;
#endif

    // アフィン変換の実行
    cv::warpAffine(source_img_, output_img_, matrix, img_size, interepolation, borderMode, borderValue);

    return;
}

cv::Mat AffineTrans::calcShiftMatrix(double tx, double ty)
{
#if DEBUG
    ROS_INFO("DEBUG : Shift info");
    std::cout << "x: " << tx << std::endl;
    std::cout << "y: " << ty << std::endl;
#endif

    return (cv::Mat_<double>(2,3)<<1.0, 0.0, tx, 0.0, 1.0, ty);
}

cv::Mat AffineTrans::calcRotateMatrix(cv::Point2f rotation_center, double rotation_angle, double scale)
{
#if DEBUG
    ROS_INFO("DEBUG : Rotation info");
    std::cout << "rotation center coord: " << rotation_center << std::endl;
    std::cout << "angle(degree): " << rotation_angle << std::endl;
    std::cout << "scale: " << scale << std::endl;
#endif
    return (cv::getRotationMatrix2D(rotation_center, rotation_angle_, scale));
}