/**
* @file     affine_trans_of_map.hpp
* @brief    地図変換実行クラスAffineTransOfMapのヘッダファイル
* @author   S.Kumada
* @date     2023/06/18
* @note     地図を変換するための各処理を行うクラスの定義
*/

#ifndef _AFFINE_TRANS_OF_MAP_H_ 
#define _AFFINE_TRANS_OF_MAP_H_ 

#include <ros/ros.h> // 基本ライブラリ

#include <nav_msgs/OccupancyGrid.h> // 地図データ取り扱い
#include <geometry_msgs/PoseWithCovarianceStamped.h> // 姿勢情報取り扱い
#include <geometry_msgs/PoseStamped.h> // 姿勢情報取り扱い
#include "uoa_poc6_msgs/r_map_pose_correct_info.h"

#include "utilitie.h"

/**
 * @brief 補正値の格納用構造体
 */
typedef struct CorrectValueInfo
{  // 補正値情報
    Vector2d shift_amount;                  //!< 平行移動量
    geometry_msgs::Point rotation_center;   //!< 回転中心座標
    double rotation_angle;                  //!< 回転角度
}stCorrectValueInfo;

/**
 * @brief 出力地図情報の格納用構造体
 */
typedef struct OutputMapInfo
{
    unsigned int map_width;         //!< 出力する地図の幅
    unsigned int map_height;        //!< 出力する地図の高さ
    geometry_msgs::Pose origin;     //!< 出力する地図の原点
    std::string map_frame;          //!< 地図のフレームID
}stOutputMapInfo;

/**
 * @brief 地図変換クラス
 * @details 取得した地図を補正値をもとに変換を実行する
 */
class AffineTransOfMap
{
    public:

    /**
    * @brief   AffineTransOfMapクラスのコンストラクタ
    * @details 初期化を行う
    */
    AffineTransOfMap(ros::NodeHandle &node);

    /**
    * @brief   AffineTransOfMapクラスのデストラクタ
    * @details オブジェクトの破棄を行う
    */
    ~AffineTransOfMap();

    /**
     * @brief        メートル座標成分をセル座標成分への変換関数
     * @param[in]    double meters 距離[m]
     * @return       void
     * @details      座標成分をメートルからセルへ変換する
     */
    inline int metersToCell(double meters)
    {
        return (int)round(meters / resolution_);
    }

    /**
     * @brief        セル座標成分をメートル座標成分への変換関数
     * @param[in]    int cell セル座標[cell]
     * @return       void
     * @details      座標成分をセルからメートルへ変換する
     */
    inline double cellToMeters(int cell)
    {
        return cell * resolution_;
    }

    /**
     * @brief        メインループ関数
     * @param[in]    void
     * @return       void
     * @details      トピックの受信待機ループ処理を行う
     */
    void affineTranceLoop();

    private:
    /**
     * @brief        合わせる対象の地図の受信コールバック関数
     * @param[in]    nav_msgs::OccupancyGrid map ターゲット地図
     * @return       void
     * @details      変換する際に最終的に合わせる対象のレイヤ地図データを受信する
     */
    void recvTargetMapCB(const nav_msgs::OccupancyGridConstPtr &map);

    /**
     * @brief        変換する対象の地図受信コールバック関数
     * @param[in]    nav_msgs::OccupancyGrid map ソース地図
     * @return       void
     * @details      変換対象の地図データを受信する
     */
    void recvSourceMapCB(const nav_msgs::OccupancyGridConstPtr &map);
    
    /**
     * @brief        地図の補正値情報受信コールバック関数
     * @param[in]    uoa_poc6_msgs::r_put_map_pose_correct &correct_info 補正値情報
     * @return       void
     * @details      変換した地図の座標系を合わせるための、環境地図の補正値情報を受信する
     */
    void recvCorrectInfoCB(const uoa_poc6_msgs::r_map_pose_correct_info &correct_info);

    /**
     * @brief        変換処理実行処理関数
     * @param[in]    void
     * @return       void
     * @details      変換処理を実行する
     */
    void runConvert();

    double max_angle_;  // 最大角度
    double angle_resolution_;   // 回転角度の分解能
    double start_angle_;    // 最初の角度
    double resolution_;    // 地図の解像度
    int intensity_; // 不明領域の設定値
    stCorrectValueInfo correct_info_;   // 受信した補正値
    stOutputMapInfo output_map_info_;   // 出力する地図の情報
    unsigned int tranceform_mode_;  // 回転方向
    bool is_get_correct_value_for_topic_; // 補正値の取得先をトピックから取得するか
    bool is_get_outmap_info_for_topic_; // 出力地図の情報をトピックから取得するか
    bool is_recv_target_;   // ターゲット地図の取得フラグ
    bool is_recv_source_;   // ソース地図の取得フラグ
    bool is_recv_correct_value_;   // 補正値のの取得フラグ
    bool is_use_center_conf_val_;   // 回転中心座標のコンフィグ値指定フラグ
    bool is_continue_rotation_; // 連続回転の使用フラグ
    ros::Time correct_value_update_time_; // 補正値の更新時間
    
    ros::Publisher  pub_result_map_;    // 補正した地図のパブリッシャ
    ros::Subscriber sub_source_map_;    // 補正する対象の地図のサブスクライバ
    ros::Subscriber sub_target_map_;    // 補正先の地図のサブスクライバ
    ros::Subscriber sub_correct_info_;  // 補正値情報のサブスクライバ

    nav_msgs::OccupancyGrid source_layer_map_;  // 補正する地図
    nav_msgs::OccupancyGrid output_map_;    // 補正した地図
    geometry_msgs::Pose recv_correct_config_;   // 取得した補正値

};

#endif