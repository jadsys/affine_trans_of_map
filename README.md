affine_trans_of_map
=======

概要
=======
このパッケージは同じ場所を示した（地図の更新等によって）座標系の異なる二次元格子地図（GridMap）同士をどちらか一方に合わせる（変換する地図をSource、合わせこむ先の地図をTarget）の座標の補正を行うために開発されました。
入力された同一地点の差分情報（位置と姿勢）を使い、アフィン変換にて座標系を補正した地図の生成を行います。

インストール方法
=======
### 1．ROSワークスペースのディレクトリに移動し、リポジトリをクローン
```bash 
cd ~/{ROSワークスペースディレクトリ}/src/
git clone -b "2023年度成果物" https://github.com/jadsys/affine_trans_of_map.git
```
### 2．Buildを行う
```bash 
cd affine_trans_of_map
catkin build --this
```
### X. 依存関係の解決
当パッケージでは外部パッケージとして以下を利用しております。
- [OpenCV](https://github.com/opencv/opencv)
- [uoa_poc6_msgs](https://github.com/jadsys/uoa_poc6_msgs.git)

それぞれのインストールを行います。
```bash
# vcsツールのインストール（既にインストール済みの場合スキップ）
sudo pip install -U vcstool

# rosdepのインストール
sudo apt install python3-rosdep
sudo rosdep init # 過去に実行済みの場合は実行不要
rosdep update # 過去に実行済みの場合は実行不要

# 依存関係のインストール
cd ~/{ROSワークスペースディレクトリ}/src
vcs import  < affine_trans_of_map/dependency.rosinstall
rosdep install -i --from-paths affine_trans_of_map
catkin build
```

ライセンス
=======
## BSD 3-Clause License

Copyright (c) 2023, Japan Advanced System,Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software 
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* * *
## 使用ライブラリ関係
### [OpenCV](https://github.com/opencv/opencv)
 [Apache 2.0 License](https://opensource.org/licenses/Apache-2.0)