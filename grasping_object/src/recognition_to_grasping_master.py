#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import actionlib
from enum import Enum
from std_msgs.msg import Bool, String, Float64

from happymimi_recognition_msgs.msg import RecognitionProcessingAction, RecognitionProcessingGoal
from happymimi_manipulation_msgs.msg import GraspingObjectAction, GraspingObjectGoal
from happymimi_manipulation_msgs.srv import RecognitionToGrasping

class ResultState(Enum):
    success = 1   # 成功
    wait = 2      # 待機
    failure = 3   # 失敗

# 物体認識をするクラス
class RecognitionAction(object):
    def __init__(self):
        self.recognition_feedback = ResultState.wait  # アクション通信のフィードバック(print(ResultState.wait) >>> ResultState.wait)

    # アクションサーバーのフィードバックメソッド
    def recognitionFeedback(self,msg):
        rospy.loginfo('feedback : %s'%(msg))  # フィードバックを出力(bool)
        self.recognition_feedback = ResultState.success if msg.recognize_feedback else ResultState.failure  # 真の処理 if 条件 else 偽の処理

    # 物体認識をするメソッド
    def recognizeObject(self, request):  # 引数は bottle とか cup とか etc...
        act = actionlib.SimpleActionClient('/recognition/action', RecognitionProcessingAction)  # アクションサーバ
        act.wait_for_server(rospy.Duration(5))  # アクションサーバーからの応答を待つ(5秒でタイムアウト)

        goal = RecognitionProcessingGoal()  # 独自型で初期化
        goal.target_name = request.target_name  # 把持対象の名前 
        goal.sort_option = request.sort_option # 複数のデータが並んでたら指定通りの順番にするオプション('cup'の左から2番目とか)
        act.send_goal(goal, feedback_cb = self.recognitionFeedback)  # ゴールをアクションサーバーに送信

        act.wait_for_result()  # アクションサーバーからの結果待ち
        result = act.get_result()  # アクションサーバーからの結果を代入
        print(result.result_flg)  # bool

        return result.result_flg, result.centroid_point  # bool, 3次元位置推定値

# 物体把持をするクラス
class GraspingAction(object):
    def __init__(self):
        pass

    def graspingFeedback(self,msg):
        rospy.loginfo('feedback %s'%(msg))

    def graspObject(self, target_centroid):
        act = actionlib.SimpleActionClient('/manipulation/grasp', GraspingObjectAction)  # 物体把持のアクションクライアントを宣言
        rospy.loginfo('waiting for grasping server')
        act.wait_for_server(rospy.Duration(5))  # アクションサーバーからの応答待ち(5秒でタイムアウト)
        rospy.loginfo('send goal')
        goal = GraspingObjectGoal()  # 独自型で初期化
        goal.goal = target_centroid  # 3次元位置推定値
        act.send_goal(goal, feedback_cb = self.graspingFeedback)  # サーバーにゴールを送信
        act.wait_for_result()  # アクションサーバーからの結果待ち
        result = act.get_result()  # アクションサーバーからの結果を代入

        return result.result  # bool 

# 物体認識から物体把持をするメイン関数
def actionMain(request):  # 引数は bottle とか cup とか ect...
    endeffector_pub = rospy.Publisher('/servo/endeffector',Bool,queue_size=1)  # eefのパブリッシャを宣言
    head_pub = rospy.Publisher('/servo/head',Float64,queue_size=1)  # 首のパブリッシャを宣言
    rospy.sleep(0.2)

    endeffector_pub.publish(False)  # eefをOPEN
    head_pub.publish(25.0)  # 首を下げる
    rospy.sleep(2.0)

    recognition_flg = True  # 物体認識が成功したか否かを格納する変数
    grasp_flg = False  # 物体把持が成功したか否かを格納する変数
    grasp_count = 0  # 物体把持の回数を格納する変数

    RA = RecognitionAction()
    GA = GraspingAction()

    # 物体認識と物体把持が成功するまで繰り返す(物体認識：True, 物体把持：False, 把持回数：2未満)
    while recognition_flg and not grasp_flg and grasp_count < 2 and not rospy.is_shutdown():
        rospy.loginfo('\n----- Recognition Action -----')
        recognition_flg, object_centroid = RA.recognizeObject(request)  # 物体認識
        if recognition_flg:  # 物体認識が成功したら真
            rospy.loginfo('\n-----  Grasping Action   -----')
            grasp_flg = GA.graspObject(object_centroid)  # 物体把持
            grasp_count += 1
    manipulation_flg = recognition_flg and grasp_flg  # 物体認識と物体把持が成功した場合のみTrue
    return manipulation_flg


if __name__ == '__main__':
    rospy.init_node('manipulation_master')
    rospy.Service('/recognition_to_grasping', RecognitionToGrasping, actionMain)  # 物体認識から物体把持の一連をするサービスサーバ
    rospy.spin()
