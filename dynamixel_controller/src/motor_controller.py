#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import rosparam
import numpy
import math
import threading #並列処理のために
import time
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message --
from happymimi_msgs.srv import StrTrg
from happymimi_recognition_msgs.srv import PositionEstimator
from happymimi_manipulation_msgs.srv import ArmControl

class MotorController(object):
    def __init__(self):
        #モータの角度、速度を取得
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,self.getMotorStateCB)
        #モータ動かすパブリッシャー
        self.motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
        #全モータの角度を出力するパブリッシャ
        self.motor_angle_pub = rospy.Publisher('/servo/angle_list',Float64MultiArray,queue_size=10)
        #モータ動かすサービスクライアント
        self.motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        # -- Motor Parameters -- (mimi_specification)
        #地面とアームを水平にしたときの角度値
        self.origin_angle = rosparam.get_param('/mimi_specification/Origin_Angle')
        #ギア比
        self.gear_ratio = rosparam.get_param('/mimi_specification/Gear_Ratio')
        self.current_pose = [0]*6 #[0, 0, 0, 0, 0, 0]
        self.torque_error = [0]*6
        self.rotation_velocity = [0]*6

        #2秒ごとにself.motorAnglePubを起動
        rospy.Timer(rospy.Duration(0.5), self.motorAnglePub)

    # 現在のモータの「角度、速度、外乱(?)」を検出
    def getMotorStateCB(self, state):
        for i in range(len(state.dynamixel_state)): #len -> モータの数
            self.current_pose[i] = state.dynamixel_state[i].present_position #角度値(パルス数)
            self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity) #速度
            self.torque_error[i] = state.dynamixel_state[i].present_current #外乱(?)

    # モータの角度パブリッシュ
    def motorAnglePub(self, event):
        # パルス値から度数法に変換
        deg_origin_angle = map(self.stepToDeg, self.origin_angle)
        deg_current_pose = map(self.stepToDeg, self.current_pose)
        # (現在のパルス値) - (オリジンパルス値) = 現在のモータのパルス値(オリジン基準)
        current_deg_list = [x-y for (x,y) in zip(deg_current_pose, deg_origin_angle)]
        current_deg_list = [round(x, 1) for x in current_deg_list] # 少数第2位を四捨五入
        current_deg_list[2] *= -1 
        current_deg_list[5] *= -1
        # Float64MultiArray => リストのメッセージ型
        pub_deg_list = Float64MultiArray(data=current_deg_list)
        self.motor_angle_pub.publish(pub_deg_list)

    def degToStep(self,deg):
        # degを4095段階のstep(12bit)に変換
        return int((deg+180)/360.0*4095)

    def stepToDeg(self,step):
        return round(step/4095.0*360.0-180, 1)

    def radToStep(self,rad):
        return int((rad + math.pi) / (2*math.pi) * 4095)

    def stepToRad(self,step):
        return step / 4095.0 * 2*math.pi - math.pi # - math.pi => 原点が180°だから0°にしたい

    # モータを動かす
    def motorPub(self, joint_name, joint_angle, execute_time=0.8):
        # Publishするデータの定義
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = joint_name
        #msg.points = [JointTrajectoryPoint() for i in range(1)]
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = joint_angle
        msg.points[0].time_from_start = rospy.Time(execute_time)
        self.motor_pub.publish(msg)

    #このスクリプト内だと、motor_id=4or5, position_value
    def setPosition(self, motor_id, position_value):
        if type(position_value) == type(float()):
            rotate_value = self.degToStep(positionon_value)
        # 目標角度を指定
        res = self.motor_client('', motor_id, 'Goal_Position', position_value)

    #このスクリプト内だと、motor_id=4, current_value=200
    def setCurrent(self, motor_id, current_value):
        # 許容電圧を指定
        res = self.motor_client('', motor_id, 'Goal_Current', current_value)


class JointController(MotorController):
    def __init__(self):
        super(JointController,self).__init__()
        rospy.Subscriber('/servo/shoulder',Float64,self.controlShoulder)
        rospy.Subscriber('/servo/elbow',Float64,self.controlElbow)
        rospy.Subscriber('/servo/wrist',Float64,self.controlWrist)
        rospy.Subscriber('/servo/endeffector',Bool,self.controlEndeffector)
        rospy.Subscriber('/servo/head',Float64,self.controlHead) #-30~40[deg]の範囲

    # 変換処理
    def shoulderConversionProcess(self, deg):
        deg *= self.gear_ratio[0] #指定の角度*2.1(ギア比)
        rad = math.radians(deg) #度数法からラジアンに変換
        print 'rad: ', rad
        #m0_rad = math.pi - rad + self.stepToRad(self.origin_angle[0])
        # originが基準だから、肩はoriginが90°(仮定)、つまり、originから30°動かしたかったら90°-30°=60°動かす
        m0_rad = -1*rad + self.stepToRad(self.origin_angle[0])
        m1_rad = rad + self.stepToRad(self.origin_angle[1])
        print 'm0_origin', self.stepToRad(self.origin_angle[0])
        print 'm1_origin', self.stepToRad(self.origin_angle[1])
        return m0_rad, m1_rad

    def elbowConversionProcess(self, deg):
        rad = math.radians(deg)
        m2_rad = rad + self.stepToRad(self.origin_angle[2])
        print 'm2_origin', self.stepToRad(self.origin_angle[2])
        return m2_rad

    def wristConversionProcess(self, deg):
        rad = math.radians(deg)
        m3_rad = rad + self.stepToRad(self.origin_angle[3])
        print 'm3_origin', self.stepToRad(self.origin_angle[3])
        return m3_rad

    def controlShoulder(self,deg):
        try:
            deg = deg.data
        except AttributeError:
            pass
        m0, m1 = self.shoulderConversionProcess(deg)
        self.motorPub(['m0_shoulder_left_joint', 'm1_shoulder_right_joint'], [m0, m1])

    def controlElbow(self,deg):
        try:
            deg = deg.data
        except AttributeError:
            pass
        m2 = self.elbowConversionProcess(deg)
        self.motorPub(['m2_elbow_joint'], [m2])

    def controlWrist(self,deg):
        try:
            deg = deg.data
        except AttributeError:
            pass
        m3 = self.wristConversionProcess(deg)
        self.motorPub(['m3_wrist_joint'], [m3])

    def controlEndeffector(self,req):
        try:
            req = req.data
        except AttributeError:
            pass

        # OPEN
        if not req:
            self.setCurrent(4, 200)
            self.setPosition(4, self.origin_angle[4])
            rospy.sleep(0.2)
            return True

        # CLOSE
        goal_position = self.origin_angle[4] + 480
        self.setCurrent(4, 200)
        self.setPosition(4, goal_position)
        rospy.sleep(0.2)

        while self.rotation_velocity[4] > 0 and not rospy.is_shutdown():
            pass
        else:
            rospy.sleep(0.5)
            #self.setPosition(4, self.current_pose[4])
        grasp_flg = self.torque_error[4] > 30
        print grasp_flg
        return grasp_flg

    def controlHead(self,deg):
        try:
            deg = deg.data
        except AttributeError:
            pass
        deg *= self.gear_ratio[5] #ギア比: 1
        step = self.degToStep(deg) + (self.origin_angle[5]-2048)
        self.setPosition(5, step)


class ManipulateArm(JointController):
    def __init__(self):
        super(ManipulateArm,self).__init__()
        rospy.Service('/servo/arm', StrTrg, self.changeArmPose) #originとかcarryとか...
        rospy.Service('/servo/debug_arm', ArmControl, self.armControlService) #わからん
        self.detect_depth = rospy.ServiceProxy('/detect/depth', PositionEstimator) #物体の位置推定?
        self.arm_specification = rosparam.get_param('/mimi_specification')

    # 逆運動学
    def inverseKinematics(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        l0 = self.arm_specification['Ground_Arm_Height'] #アームの高さ
        l1 = self.arm_specification['Shoulder_Elbow_Length'] #肩から肘の長さ
        l2 = self.arm_specification['Elbow_Wrist_Length'] #肘から手首の長さ
        l3 = self.arm_specification['Wrist_Endeffector_Length'] #手首からエンドエフェクタの長さ
        x -= l3
        y -= l0

        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)

        try:
            #肩
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
            #肘
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
            #手首
            wrist_angle = -1*(shoulder_angle + elbow_angle)
            angle_list = [shoulder_angle, elbow_angle, wrist_angle]
            #angle_listを度数法に変換
            angle_list = map(math.degrees, angle_list)
            return angle_list
        except ValueError:
            rospy.loginfo('can not move arm.')
            return [numpy.nan]*3

    #アームを動かす(armControllerBytopicで並列処理できるからここは使われてない)
    def armController(self, joint_angle):
        try:
            joint_angle = joint_angle.data
        except AttributeError:
            pass
        thread_shoulder = threading.Thread(target=self.controlShoulder, args=(joint_angle[0],))
        thread_elbow = threading.Thread(target=self.controlElbow, args=(joint_angle[1],))
        thread_wrist = threading.Thread(target=self.controlWrist, args=(joint_angle[2],))
        thread_wrist.start()
        #rospy.sleep(0.5)
        thread_elbow.start()
        #rospy.sleep(0.5)
        thread_shoulder.start()

    #アームをトピックで動かす
    def armControllerByTopic(self, joint_angle): #joint_angleは逆運動学の計算結果
        # Originの位置から各モータを何度動かすか計算する
        m0, m1 = self.shoulderConversionProcess(joint_angle[0])
        m2 = self.elbowConversionProcess(joint_angle[1])
        m3 = self.wristConversionProcess(joint_angle[2])

        print 'm0, m1, m2, m3'
        print m0, m1, m2, m3
        print map(math.degrees, [m0, m1, m2, m3]) #度数法に変換
        self.motorPub(['m0_shoulder_left_joint', 'm1_shoulder_right_joint', 'm2_elbow_joint', 'm3_wrist_joint'], [m0, m1, m2, m3]) #アームを動かす

    def armControlService(self, coordinate): #サービスクライアントからの引数
        try:
            coordinate = coordinate.data
        except AttributeError:
            pass
        joint_angle = self.inverseKinematics(coordinate)
        print ''
        print 'joint_angle'
        print joint_angle
        #非数かどうかを判定
        if numpy.nan in joint_angle: #非数かどうか確認
            return False
        #self.armController(joint_angle)
        self.armControllerByTopic(joint_angle)
        return True

    def changeArmPose(self, cmd): #cmd: サービスクライアントからの引数(originとかcarryとか)
        if type(cmd) != str:
            cmd = cmd.data
        rospy.loginfo('Change arm command : %s'%cmd)
        if cmd == 'origin':
            self.originMode()
            return True
        elif cmd == 'carry':
            self.carryMode()
            return True
        elif cmd == 'receive':
            res = self.receiveMode()
            return res
        elif cmd == 'give':
            self.giveMode()
            return True
        elif cmd == 'place':
            res = self.placeMode()
            return res
        else :
            rospy.loginfo('No such change arm command.')
            return False

    def originMode(self):
        shoulder_param = 0
        elbow_param = 0
        wrist_param = 0
        #self.armController([shoulder_param, elbow_param, wrist_param])
        self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])

    def carryMode(self):
        shoulder_param = -85
        elbow_param = 90
        wrist_param = 90
        #self.armController([shoulder_param, elbow_param, wrist_param])
        self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])

    def receiveMode(self):
        self.controlHead(25)
        rospy.sleep(0.5)
        shoulder_param = -40
        elbow_param = 70
        wrist_param = -30
        #self.armController([shoulder_param, elbow_param, wrist_param])
        self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])
        rospy.sleep(0.5)
        self.controlEndeffector(False)

        rospy.wait_for_service('/detect/depth')
        endeffector_res = False
        count = 0
        '''
        while not endeffector_res and count<2 and not rospy.is_shutdown():
            self.controlEndeffector(False)
            rospy.sleep(2.0)
            count += 1
            start_time = time.time()
            straight_line_distance = 9.99
            while time.time()-start_time<3.0 and straight_line_distance>0.42 and not rospy.is_shutdown():
                depth_res = self.detect_depth(280, 360)
                straight_line_distance = depth_res.point.x
            rospy.sleep(0.5)
            endeffector_res = self.controlEndeffector(True)
            rospy.sleep(1.5)
        '''
        self.controlEndeffector(False)
        rospy.sleep(2.0)
        start_time = time.time()
        straight_line_distance = 9.99
        while time.time()-start_time<3.0 and straight_line_distance>0.42 and not rospy.is_shutdown():
            depth_res = self.detect_depth(280, 365) #物体の位置推定？
            straight_line_distance = depth_res.point.x
        rospy.sleep(0.5)
        endeffector_res = self.controlEndeffector(True)
        rospy.sleep(1.5)

        self.carryMode()
        self.controlHead(0)
        return endeffector_res

    def giveMode(self):
        shoulder_param = -35
        elbow_param = 75
        wrist_param = -35
        #self.armController([shoulder_param, elbow_param, wrist_param])
        self.armControllerByTopic([shoulder_param, elbow_param, wrist_param])
        rospy.sleep(4.0)
        rospy.loginfo('give!!')
        '''
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass

        rospy.sleep(1.0)
        '''
        wrist_error = abs(self.torque_error[3])
        give_time = time.time()
        while abs(wrist_error - abs(self.torque_error[3])) < 10 and time.time() - give_time < 5.0 and not rospy.is_shutdown():
            pass
        self.setPosition(4, self.origin_angle[4])
        rospy.sleep(0.5)
        self.carryMode()

    def placeMode(self):
        #現時点では家具の高さを予めプログラムに打ち込む必要があり、
        #その情報をobject_grasperに格納しているのでそちらでplaceの関数をオーバーライドしています。
        pass


if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = ManipulateArm()
    rospy.spin()
