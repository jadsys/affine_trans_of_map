/**
* @file     affine_trans_of_map.cpp
* @brief    地図変換実行クラスAffineTransOfMapの定義ソースファイル
* @author   S.Kumada
* @date     2023/06/18
* @note     地図を変換するための各処理を行うクラスの実装
*/

#include "affine_trans_of_map/affine_trans_of_map.hpp"
#include "affine_trans_of_map/affine_trans.hpp"


AffineTransOfMap::AffineTransOfMap(ros::NodeHandle &node)
{
    ros::NodeHandle param_node("~");
    std::string sub_source_topic_name;
    std::string sub_target_topic_name;
    std::string sub_correct_info_topic_name;
    std::string pub_topic_name;
    std::string tranceform_mode;
    is_recv_target_ = false;
    is_recv_source_ = false;
    is_recv_correct_value_ = false;
    
    // パラメータ読み込み
    // アフィン変換を実行する対象の地図トピック
    getParam(param_node, "source_map_topic_name",   sub_source_topic_name,    std::string("/map"));

    // ターゲットとなる地図トピック
    getParam(param_node, "target_map_topic_name",   sub_target_topic_name,    std::string("/map"));

    // アフィン変換後の地図トピック
    getParam(param_node, "converted_map_topic_name", pub_topic_name,    sub_source_topic_name + std::string("_converted"));
    
    // 地図の補正値情報トピック
    getParam(param_node, "correct_info_topic_name", sub_correct_info_topic_name,    std::string("/map_correct_info"));
    
    // 変換方向
    getParam(param_node, "tranceform_mode", tranceform_mode,    std::string("forward"));
    if(tranceform_mode == "inverse")
    { // 逆変換の場合

#if DEBUG
        ROS_INFO("DEBUG : Tranceform mode inverse");
#endif
        tranceform_mode_ = TRANCEFORM_MODE_INVERSE;
    }
    else
    { // それ以外
#if DEBUG
        ROS_INFO("DEBUG : Tranceform mode forward");
#endif
        tranceform_mode_ = TRANCEFORM_MODE_FOWARD;
    }
 
    // トピックから取得するか
    getParam(param_node, "config_from_topic/correct_value", is_get_correct_value_for_topic_,    false);   // 補正値情報
    getParam(param_node, "config_from_topic/output_map", is_get_outmap_info_for_topic_,    false);       // 出力地図情報

    // 補正値情報の有効化（無効の場合はトピックから取得する）
    // 平行移動量
    getParam(param_node, "correct_value/shift_amount/x", correct_info_.shift_amount.x,    0.0);
    getParam(param_node, "correct_value/shift_amount/y", correct_info_.shift_amount.y,    0.0);

    // 回転中心座標を指定するか
    getParam(param_node, "correct_value/rotation_center/enable", is_use_center_conf_val_,    false);

    // 回転中心座標
    getParam(param_node, "correct_value/rotation_center/x", correct_info_.rotation_center.x,    0.0);
    getParam(param_node, "correct_value/rotation_center/y", correct_info_.rotation_center.y,    0.0);

    // 回転角度
    getParam(param_node, "correct_value/rotation_angle", correct_info_.rotation_angle,    0.0);

    // 地図のフレーム名
    std::string map_frame;
    getParam(param_node, "output_map/map_frame", map_frame,    std::string("map"));
    output_map_info_.map_frame = map_frame;
    
    // 出力地図のサイズ
    int map_width, map_height;
    getParam(param_node, "output_map/size/width",   map_width,  0);
    getParam(param_node, "output_map/size/height",  map_height, 0);
    if(map_width < 0)
    {
        ROS_WARN_STREAM("Read failure param key: " << "output_map/size/width" << " / value: "<< map_width);
        map_width = 0;
    }
    else if(map_height < 0)
    {
        ROS_WARN_STREAM("Read failure param key: " << "output_map/size/height" << " / value: "<< map_height);
        map_height = 0;
    }
    output_map_info_.map_width  = map_width;
    output_map_info_.map_height = map_height;

    // 出力地図の原点位置の指定
    getParam(param_node, "output_map/origin/x", output_map_info_.origin.position.x,    0.0);
    getParam(param_node, "output_map/origin/y", output_map_info_.origin.position.y,    0.0);
    double yaw; // radian
    getParam(param_node, "output_map/origin/yaw", yaw,    0.0); 
    quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, yaw), output_map_info_.origin.orientation); // TF ⇒ geometory_msgs(RPY ⇒ Quaterninon, geometory_msgs::Quarternion)

    // 不明領域の設定値
    getParam(param_node, "intensity", intensity_,    205);

    // 連続回転の設定
    // 連続回転の使用フラグ
    getParam(param_node, "continuous_rotation_option/enable", is_continue_rotation_,    false);

    // 最大回転角度
    getParam(param_node, "continuous_rotation_option/max_angle", max_angle_,    360.0);

    // 回転角度の分解能
    getParam(param_node, "continuous_rotation_option/angle_resolution", angle_resolution_,    10.0);

    // 初期角度の設定
    getParam(param_node, "continuous_rotation_option/start_angle", start_angle_,    0.0);


    // パブリッシャサブスクライバ定義
    pub_result_map_ = node.advertise<nav_msgs::OccupancyGrid>(pub_topic_name, ROS_QUEUE_SIZE_1, true);
    sub_source_map_ = node.subscribe(sub_source_topic_name,  ROS_QUEUE_SIZE_1, &AffineTransOfMap::recvSourceMapCB, this);
    if(is_get_outmap_info_for_topic_)
    { // トピックから取得する場合
        sub_target_map_ = node.subscribe(sub_target_topic_name,  ROS_QUEUE_SIZE_1, &AffineTransOfMap::recvTargetMapCB, this);
    }
    if(is_get_correct_value_for_topic_)
    { // トピックから取得する場合
        sub_correct_info_ = node.subscribe(sub_correct_info_topic_name,  ROS_QUEUE_SIZE_1, &AffineTransOfMap::recvCorrectInfoCB, this); 
    } 
}

AffineTransOfMap::~AffineTransOfMap()
{
    // 何もしない
}


void AffineTransOfMap::recvTargetMapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
#ifdef DEBUG
    ROS_INFO("DEBUG : Recieve TargetMap");
#endif

    output_map_info_.map_height = map->info.height;
    output_map_info_.map_width  = map->info.width;
    output_map_info_.origin     = map->info.origin;

    if(!is_use_center_conf_val_)
    { // YAMLの設定値を使用しない場合は、ターゲット地図の原点を回転中心とする
        correct_info_.rotation_center.x = (int)round(abs(map->info.origin.position.x / map->info.resolution)); // [m] ⇒[cell] 
        correct_info_.rotation_center.y = map->info.height - (int)round(abs(map->info.origin.position.y / map->info.resolution));
    }
    
    // 受信フラグをON
    is_recv_target_ = true;
}

void AffineTransOfMap::recvSourceMapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
#ifdef DEBUG
    ROS_INFO("DEBUG : Recieve SourceMap");
#endif
    // 受信データの保持
    source_layer_map_ = *map;
    resolution_ = map->info.resolution;

    // 受信フラグをON
    is_recv_source_ = true;
}

void AffineTransOfMap::recvCorrectInfoCB(const uoa_poc6_msgs::r_map_pose_correct_info &correct_info)
{
#ifdef DEBUG
    ROS_INFO("DEBUG : Recieve Correct Info");
#endif

    if(correct_info.header.stamp != correct_value_update_time_)
    {
        recv_correct_config_ = correct_info.pose;

        // 最終リビジョン番号の更新
        correct_value_update_time_ = correct_info.header.stamp;

        // 受信フラグをON
        is_recv_correct_value_ = true;
    }

}

void AffineTransOfMap::runConvert()
{
#ifdef DEBUG
    ROS_INFO("------ STEP1: Affine transformation parameter setting ------");
#endif

    AffineTrans affin;

    // 変換対象の地図のセット
    affin.setSourceImg(source_layer_map_);

    // 地図の不明領域の置き換え値の設定
    affin.setIntensity(intensity_);

    // 地図の原点位置の設定
    affin.setConvertedOriginCoord(output_map_info_.origin);

    // 地図のフレーム名の設定
    affin.setMapFrame(output_map_info_.map_frame);

    // 変換モードの設定
    // affin.setTranceformMode(tranceform_mode_t(tranceform_mode_));
    
    // トピックから受信した補正値で変換する際の前処理
    if(is_get_correct_value_for_topic_)
    { // トピックから取得した場合は原点の考慮が必要
        // Quatanion → Euler
        double yaw, pitch, roll;
        tf::Quaternion quat; 
        tf::quaternionMsgToTF(recv_correct_config_.orientation, quat);
        tf::Matrix3x3 mat(quat);
        mat.getEulerYPR(yaw, pitch, roll);
        int tranceform_sign = 1;

        // 補正情報の格納
        if(tranceform_mode_ == TRANCEFORM_MODE_INVERSE)
        {
            tranceform_sign = -1;
        }

        correct_info_.rotation_angle = tranceform_sign * rad_to_deg(yaw);

#ifdef DEBUG
        std::cout << "DEBUG : Correct info yaw " << yaw <<  " [rad]" <<  std::endl;
        std::cout << "DEBUG : Correct info angle " << correct_info_.rotation_angle << " [deg]" << std::endl;
#endif

        // ソース地図とターゲット地図の原点位置の差分
        // originは左下のセルからの距離情報
        // x軸は左から右と座標系と一致しているため、ターゲット-ソース
        double diff_origin_position_x = abs(output_map_info_.origin.position.x) - abs(source_layer_map_.info.origin.position.x);
        // y軸の座標系は反転するため、高さから原点座標を引いた値が地図の原点からの座標位置となる
        double diff_origin_position_y = (cellToMeters(output_map_info_.map_height) - abs(output_map_info_.origin.position.y)) - (cellToMeters(source_layer_map_.info.height) - abs(source_layer_map_.info.origin.position.y));
        // 平行移動成分の計算
        correct_info_.shift_amount.x = diff_origin_position_x - tranceform_sign * recv_correct_config_.position.x;
        correct_info_.shift_amount.y = diff_origin_position_y + tranceform_sign * recv_correct_config_.position.y; // 補正値は右手系、平行移動関数は左手系のため、補正値の符号反転
    }

    // 平行移動量の設定
    int shift_x = metersToCell(correct_info_.shift_amount.x);
    int shift_y = metersToCell(correct_info_.shift_amount.y);
    affin.setShiftAmount(shift_x, shift_y);
    
#ifdef DEBUG
    ROS_INFO("DEBUG : Set shift amount @ x=%f y=%f [m] | x=%d, y=%d [cell]", correct_info_.shift_amount.x, correct_info_.shift_amount.y, shift_x, shift_y);
#endif

    // 平行移動後の地図のサイズ設定
    affin.setConvertedImgSize(output_map_info_.map_width, output_map_info_.map_height); // 出力地図と同サイズ

#ifdef DEBUG
    ROS_INFO("------ STEP2: Run shift transformation ------");
#else 
    ROS_INFO("------ Start of affine transformation ------");
#endif

    // 平行移動変換
    affin.shiftConvert();

    // 回転
    // 平行移動変換の結果をインプットとする
    affin.setSourceImg(affin.getResultImg());

    // 回転中心座標の設定
    affin.setRotationCenter(correct_info_.rotation_center.x, correct_info_.rotation_center.y);

    // 回転角度の設定
    affin.setRotationYaw(correct_info_.rotation_angle);

    // 回転後の地図サイズを設定
    // affin.setConvertedImgSize(output_map_info_.map_width, output_map_info_.map_height); 

#ifdef DEBUG
    ROS_INFO("------ STEP3: Run rotate transformation ------");
#endif

    // 回転変換
    affin.rotateConvert();

#ifdef DEBUG
    ROS_INFO("DEBUG : Center of rotation @ x=%f y=%f", correct_info_.rotation_center.x, correct_info_.rotation_center.y);
    // 確認用imshow
    // affin.DebugImgShow("Shifted");
#endif

#ifdef DEBUG
    ROS_INFO("------ STEP4: Get result map ------");
#else 
    ROS_INFO("------ End of affine transformation ------");
#endif

    // 結果の取得
    output_map_ = affin.getResultImg();

}


void AffineTransOfMap::affineTranceLoop()
{

    ros::Rate sleep_rate = ROS_RATE_30HZ; // Hz

    while(ros::ok)
    {
        sleep_rate.sleep(); // Sleep
        ros::spinOnce();    // コールバック関数実行

        if( is_recv_target_ && 
            (!is_get_correct_value_for_topic_ || is_recv_correct_value_)  &&  // 補正値情報をトピックから取得するかつ、情報が設定された場合 
            (!is_get_outmap_info_for_topic_ || is_recv_source_))            // 出力地図の情報をトピックから取得するかつ、情報が設定された場合
        {
            // 変換処理実行prev_time_
            runConvert();
            
#ifdef DEBUG
    ROS_INFO("------ STEP5: Pub transformated map ------");
#endif
            // 変換後の地図データの配信
            pub_result_map_.publish(output_map_);

            // 受信フラグオフ
            is_recv_target_         = false;
            is_recv_correct_value_  = false;
            is_recv_source_         = false;

        }
    }
    
    return;
}
