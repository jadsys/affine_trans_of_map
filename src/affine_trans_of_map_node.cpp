/**
* @file     affine_trans_of_map_node.cpp
* @brief    affine_trans_of_mapパッケージのmain関数
* @author   S.Kumada
* @date     2023/06/16
* @note     ノードの初期化、変換処理の初期化を行う
*/

#include "affine_trans_of_map/affine_trans_of_map.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "affine_trans_of_map");
    ros::NodeHandle node;

    AffineTransOfMap trans(node);
    trans.affineTranceLoop();

    return 0;
}