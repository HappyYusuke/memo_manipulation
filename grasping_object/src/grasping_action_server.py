#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import rosparam
import roslib.packages
import os
import sys
import time
import math
import numpy
import threading
import actionlib
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.msg import *

motor_controller_path = roslib.packages.get_pkg_dir('dynamixel_controller')
sys.path.insert(0, os.path.join(motor_controller_path, 'src/'))
from motor_controller import ManipulateArm

teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

class GraspingActionServer(ManipulateArm):
    def __init__(self):
        super(GraspingActionServer,self).__init__()
        rospy.Subscriber('/current_location',String,self.navigationPlaceCB)   # Navigationが成功したときのロケーションネームをサブスクライブ 
                                                                              # *詳しくは /happymimi_apps/happymimi_navigation/src/navi_location.py
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)   # base_control用のパブリッシャ
        self.navigation_place = 'Null'   # 現在位置格納用変数
        self.target_place = rosparam.get_param('/location_dict')   # 各オブジェクトの高さのディクショナリ *詳しくは /grasping_object/param/location_dict.yaml

        self.base_control = BaseControl()

        self.act = actionlib.SimpleActionServer('/manipulation/grasp',         # アクションサーバ定義
                                                GraspingObjectAction,          # 独自型 /happymimi_manipulation_msgs/action/GraspingObject.action
                                                execute_cb = self.actionMain,  # コールバック関数を指定
                                                auto_start = False)            #サーバの自動起動を無効にするためにFalseを指定
        self.act.register_preempt_callback(self.actionPreempt)                 # 実行中のプログラムを強制的に一時中断し、他のプログラムを実行する

        self.act.start()   # アクションサーバ起動

    def placeMode(self):#override
        self.base_control.translateDist(-0.15)                # 少し後退する
        rospy.sleep(1.0)
        y = self.target_place[self.navigation_place] + 0.14   # Navigationに成功したときの家具の高さの座標
        #x = (y-0.78)/10+0.5
        x = 0.5                                               # placeモード時のマニピュレータのx方向の長さ
        joint_angle = self.inverseKinematics([x, y])          # 逆運動学
        if numpy.nan in joint_angle:                          # 非数かどうか確認
            return False

        self.armControllerByTopic(joint_angle)      # マニピュレーション
        rospy.sleep(2.5)
        self.base_control.translateDist(0.3)        # 前進
        rospy.sleep(1.0)
        self.base_control.translateDist(0.1, 0.1)   # ゆっくり前進
        rospy.sleep(1.0)

        joint_angle = self.inverseKinematics([x, y-0.03])   # 物体を置くために手先の位置を下げる計算
        if not (numpy.nan in joint_angle):                  # 非数ではなかったらTrue
            self.armControllerByTopic(joint_angle)          # 手先の位置を少し下げるマニピュレーション
        rospy.sleep(2.5)
        self.controlEndeffector(False)           # エンドエフェクタを開く
        rospy.sleep(2.0)
        self.base_control.translateDist(-0.25)   # 後退する
        self.changeArmPose('carry')              # carryモード
        self.navigation_place = 'Null'           # Nullに戻す
        rospy.loginfo('Finish place command\n')
        return True

    def approachObject(self,object_centroid):   # 引数はオブジェクトの重心の座標 
        if object_centroid.x > 1.5:   # オブジェクトまでの距離が1.5[m]だったらTrue
            return False
        elif object_centroid.x < 0.5 or object_centroid.x > 0.8:   # オブジェクトまでの距離が x < 0.5 か 0.8 < x だったらTrue
            move_range = (object_centroid.x-0.65)                  # オブジェクトに接近する距離（以下、接近距離）
            if abs(move_range) < 0.2:                              # 接近距離の絶対値が0.2[m]未満ならTrue
                move_range = int(move_range/abs(move_range))*0.2   # 
            self.base_control.translateDist(move_range)            # オブジェクトに接近
            rospy.sleep(4.0)
            return False
        else :
            return True

    def graspObject(self, object_centroid):
        rospy.loginfo('\n----- Grasp Object -----')

        x = 0.475                      # 物体把持時のマニピュレータのx方向の長さ
        #x = (y-0.75)/10+0.5
        y = object_centroid.z + 0.03   # オブジェクトの高さ
        '''
        if self.navigation_place == 'Null':
            y = object_centroid.z + 0.05
        else:
            y = self.target_place[self.navigation_place] + 0.10
        '''
        joint_angle = self.inverseKinematics([x, y])   # 逆運動学
        if numpy.nan in joint_angle:                   # 非数か確認
            return False
        self.armControllerByTopic(joint_angle)         # 物体把持のフォームにマニピュレーション
        rospy.sleep(2.5)

        move_range = object_centroid.x + 0.07 - x               # 物体に接近する距離
        self.base_control.translateDist(move_range*0.8, 0.15)   # 物体に接近
        rospy.sleep(0.5)
        self.base_control.translateDist(move_range*0.2, 0.1)    # 物体にゆっくり接近
        rospy.sleep(0.5)

        grasp_flg = self.controlEndeffector(True)   # エンドエフェクタを閉じる
        rospy.sleep(1.0)
        self.controlWrist(joint_angle[2]+45.0)      # 手首を上方向に曲げる
        self.base_control.translateDist(-0.3)       # 後退する

        self.changeArmPose('carry')   # carryモード
        rospy.sleep(4.0)

        '''
        if grasp_flg :
            grasp_flg = abs(self.torque_error[4]) > 30
        '''
        if grasp_flg :
            rospy.loginfo('Successfully grasped the object!')
        else:
            self.setPosition(4, self.origin_angle[4])
            rospy.loginfo('Failed to grasp the object.')
        rospy.loginfo('Finish grasp.')
        return grasp_flg

    def navigationPlaceCB(self,res):
        self.navigation_place = res.data  # 現在位置を格納

    def startUp(self):
        _ = self.controlEndeffector(False)   # return値のメモリの占用をしないまま廃棄
        self.changeArmPose('carry')          # carryモード
        self.controlHead(0.0)                # 首を真っ直ぐにする

    def actionPreempt(self):
        rospy.loginfo('Preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self,object_centroid):   # 引数はアクション通信の引数 *詳しくは /happymimi_manipulation_msgs/action/GraspingObject.action
        target_centroid = object_centroid.goal   # アクション通信のゴールを代入
        grasp_result = GraspingObjectResult()    # アクション通信の結果を代入
        grasp_flg = False                                     # Falseで初期化
        approach_flg = self.approachObject(target_centroid)   # オブジェクトに接近. 引数はRealSenseからの値(オブジェクトの重心の座標)
        if approach_flg:                                      # オブジェクトへの接近が成功したらTrue
            grasp_flg = self.graspObject(target_centroid)     # 物体把持
        grasp_result.result = grasp_flg                       # 物体把持の結果を代入
        self.act.set_succeeded(grasp_result)                  # set_succeeded()でSimpleActionServerにゴールしたことを伝える 


if __name__ == '__main__':
    rospy.init_node('grasping_action_server')
    grasping_action_server= GraspingActionServer()
    grasping_action_server.startUp()
    rospy.spin()
