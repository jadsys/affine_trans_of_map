/**
* @file     affine_trans.hpp
* @brief    アフィン変換実行クラスAffineTransのヘッダファイル
* @author   S.Kumada
* @date     2023/06/18
* @note     アフィン変換により、地図画像の平行移動・回転処理を行うクラスの定義
*/

#ifndef  _AFFINE_TRANS_H_ 
#define _AFFINE_TRANS_H_ 

#include <ros/ros.h> // 基本ライブラリ

#include <nav_msgs/OccupancyGrid.h> // 地図データ取り扱い
#include <geometry_msgs/Pose.h> // 姿勢情報取り扱い
#include <tf/tf.h> // Quaternion変換
#include <opencv2/opencv.hpp> // 画像処理ライブラリ

typedef enum TRANCEFORM_MODE
{
    TRANCEFORM_MODE_INVERSE,    //!< 正の向き
    TRANCEFORM_MODE_FOWARD      //!< 逆向き
} tranceform_mode_t;

/**
 * @brief アフィン変換クラス
 * @details 任意座標における、平行移動・回転した地図を生成する
 */
class AffineTrans
{
    public:

    /**
    * @brief   AffineTransクラスのコンストラクタ
    * @details 初期化を行う
    */
    AffineTrans();

    /**
    * @brief   AffineTransクラスのデストラクタ
    * @details オブジェクトの破棄を行う
    */
    ~AffineTrans();

    /**
     * @brief        （DEBUG）出力イメージの表示関数
     * @param[in]    std::string msg タイトルバーに表示する文字列
     * @return       void
     * @details      出力イメージを確認する為のデバック関数。
     */
    void DebugImgShow(std::string msg);

    /**
     * @brief        変換元データの設定関数
     * @param[in]    nav_msgs::OccupancyGrid map_data ROSの地図形式
     * @return       void
     * @details      変換対象のOccupancyGrid形式の地図データをセットする。
     */
    void setSourceImg(nav_msgs::OccupancyGrid map_data);
    
    /**
     * @brief        変換元データの設定関数
     * @param[in]    cv::Mat source_img OpenCVのMat形式
     * @return       void
     * @details      変換対象のMat形式の画像データをセットする。
     */
    void setSourceImg(cv::Mat source_img);

    /**
     * @brief        回転の中心座標の設定関数
     * @param[in]    int coord_x X軸の座標値
     * @param[in]    int coord_y Y軸の座標値
     * @return       void
     * @details      回転の中心座標をセットする。
     */
    void setRotationCenter(int coord_x, int coord_y);

    /**
     * @brief        回転角度の設定関数
     * @param[in]    double rotation_yaw 回転角度
     * @return       void
     * @details      回転角度をセットする。
     */
    void setRotationYaw(double rotation_yaw);

    /**
     * @brief        平行移動量の設定関数
     * @param[in]    double x_axis x軸の移動量
     * @param[in]    double y_axis y軸の移動量
     * @return       void
     * @details      平行移動量の値をセットする。
     */
    void setShiftAmount(double x_axis, double y_axis);

    /**
     * @brief        変換後のサイズの設定関数
     * @param[in]    unsigned int width 幅
     * @param[in]    unsigned int height 高さ
     * @return       void
     * @details      変換後の画像サイズをセットする。
     */
    void setConvertedImgSize(unsigned int width, unsigned int height);

    /**
     * @brief        変換後のframe名の設定関数
     * @param[in]    std::string map_frame フレーム名
     * @return       void
     * @details      変換後の地図のTFフレーム名をセットする。
     */
    void setMapFrame(std::string map_frame);

    /**
     * @brief        変換後の原点情報の設定関数
     * @param[in]    geometry_msgs::Pose origin 原点情報
     * @return       void
     * @details      変換後の原点情報をセットする。
     */
    void setConvertedOriginCoord(geometry_msgs::Pose origin);

    /**
     * @brief        変換モードの設定関数
     * @param[in]    tranceform_mode_t mode 原点情報
     * @return       void
     * @details      変換後の原点情報をセットする。
     */
    void setTranceformMode(tranceform_mode_t mode);

    /**
     * @brief        拡張領域の値の設定関数
     * @param[in]    double intensity 拡張領域の値
     * @return       void
     * @details      OpenCVに変換後の、拡張した領域の値をセットする。
     */
    void setIntensity(int intensity);

    /**
     * @brief        アフィン変換後の画像データの取得関数
     * @param[in]    void
     * @return       nav_msgs::OccupancyGrid map_data アウトプット地図
     * @details      変換後のOccupancyGrid形式の地図を取得する。
     */
    nav_msgs::OccupancyGrid getResultImg(void);
    
    
    /**
     * @brief        地図データをOpenCVで扱えるように変換する関数
     * @param[in]    nav_msgs::OccupancyGrid grid_map 地図データ
     * @return       cv::Mat　地図の画像バイナリデータ
     * @details      OccupancyGrid⇒cv::Mat形式に変換する。
     */
    cv::Mat mapToMat(nav_msgs::OccupancyGrid grid_map);
    
    /**
     * @brief        cv::Mat形式の地図データをGridMap形式に変換する関数
     * @param[in]    cv::Mat mat_map 地図の画像バイナリデータ
     * @return       nav_msgs::OccupancyGrid grid_map 地図データ
     * @details      OccupancyGrid⇒cv::Mat形式に変換する。
     */
    nav_msgs::OccupancyGrid matToMap(cv::Mat mat_map);
    
    /**
     * @brief        平行移動の変換処理関数
     * @param[in]    void
     * @return       void
     * @details      平行移動のアフィン変換を行う。
     */
    void shiftConvert();

    /**
     * @brief        回転の変換処理関数
     * @param[in]    void
     * @return       void
     * @details      回転のアフィン変換を行う。
     */
    void rotateConvert();

    private:
    nav_msgs::OccupancyGrid input_map_; /// 入力地図データ
    cv::Mat source_img_; /// 変換する地図
    cv::Mat output_img_; /// 出力地図
    geometry_msgs::Point rotation_center_; /// 回転の中心座標
    geometry_msgs::Pose converted_map_origin_;    // 変換後の地図の原点情報
    double rotation_angle_; /// 回転角度
    int intensity_; /// 不明領域の値
    unsigned int width_; /// 変換後の地図の幅
    unsigned int height_; /// 変換後の地図の高さ
    double x_axis_; /// X軸の平行移動量
    double y_axis_; /// Y軸の平行移動量
    std::string map_frame_; /// 地図のフレーム名
    tranceform_mode_t tranceform_mode_; /// 回転方向

    /**
     * @brief        平行移動成分のマトリックス計算関数
     * @param[in]    double tx x軸の平行移動量
     * @param[in]    double ty y軸の平行移動量
     * @return       cv::Mat 平行移動成分の変換行列
     * @details      平行移動成分のマトリックスを計算する。
     */
    cv::Mat calcShiftMatrix(double tx, double ty);

    /**
     * @brief        回転成分のマトリックス計算関数
     * @param[in]    cv::Point2f rotate_center　回転中心座標
     * @param[in]    double rotation_angle 回転角度（degree）
     * @param[in]    double scale 拡大率
     * @return       cv::Mat 回転成分の変換行列
     * @details      回転行列を計算する。
     */
    cv::Mat calcRotateMatrix(cv::Point2f rotate_center, double rotation_angle, double scale);
    
    /**
     * @brief        アフィン変換処理関数
     * @param[in]    cv::Mat matrix
     * @param[in]    int interepolation 補完手法　デフォルト値：cv::INTER_NEAREST（最近傍補完）
     * @param[in]    int borderMode ピクセル外挿方法 デフォルト値：cv::BORDER_CONSTANT（単一色の境界）
     * @param[in]    const cv::Scalar 色　borderValue
     * @return       void
     * @details      地図データをアフィン変換する。
     */
    void affinConversion(cv::Mat matrix, int interepolation, int borderMode, const cv::Scalar borderValue);

};

#endif