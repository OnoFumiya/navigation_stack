#!/usr/bin/env python3
import rospy                                               # ROSをpythonで使うモジュール
from geometry_msgs.msg import Twist                        # ロボットに速度を発行するメッセージ型の読み込み
from nav_msgs.msg import Odometry                          # ロボットの位置を推測するメッセージ型の読み込み
import math                                                # Pythonで複雑な数学的な計算ができるモジュール


current_x = None                         # 現在地xを未確定状態(None)で取得
current_y = None                         # 現在地yを未確定状態(None)で取得
def callback_odom(msg):                  # 18行目で呼び出されている（Odometry型をmsgで受け取っていることに注意）
    global current_x, current_y          # グローバル変数の現在地の座標をcallback_odom関数内で使用する宣言
    current_x = msg.pose.pose.position.x # ロボットの現在地xをcurrent_xに更新し続ける
    current_y = msg.pose.pose.position.y # ロボットの現在地yをcurrent_yに更新し続ける


def main(vel_x, vel_y, range_m):

    vel = Twist()                           # 速度を送るための変数をTwist型で宣言
    vel.linear.x = vel_x                    # ロボットから見て並進前方方向にvel_x[m/s]の速度を変数に代入
    vel.linear.y = vel_y                    # ロボットから見て並進前方方向にvel_y[m/s]の速度を変数に代入
    msg_time = rospy.Time.now()
    now_time =  rospy.Time.now()
    last_time =  rospy.Time.now()
    last_dist = 0.0

    while not rospy.is_shutdown():                              # このプログラムがROSとして起動している間、23行目~27行目を回り続ける
        if (current_x is not None) and (current_y is not None): # ロボットの位置が、8,9行目の未確定状態から、12,13行目によって更新されれば、、、
            break                                               # このwhile文を抜け出す
    init_x = current_x                                          # 現在の位置xを初期位置として記憶する
    init_y = current_y                                          # 現在の位置yを初期位置として記憶する

    # r = rospy.Rate(100)
    while not rospy.is_shutdown():                                              # このプログラムがROSとして起動している間、29行目~34行目を回り続ける
        pub.publish(vel)                                                        # 17行目で宣言したとおり、速度を発行している
        now_time =  rospy.Time.now()
        distance = math.sqrt((current_x - init_x)**2 + (current_y - init_y)**2) # distanceに、現在の位置と、初期位置との距離を三平方から計算している
        delta_t = (now_time - last_time).to_sec()
        if delta_t > 0.1:
            delta_dist = distance - last_dist
            velocty = delta_dist / delta_t
            file_name = str("/home/sobits/catkin_ws/src/fumifumifumifumifuimifumifumi.csv")
            f = open(file_name, 'a') # 書き込みモードで開く

            txt_output_str  = str(now_time) + "," # 時間の挿入
            txt_output_str += str(distance) + "\n" # odomの挿入
            f.write(txt_output_str) # 引数の文字列をファイルに書き込む
            f.close() # ファイルを閉じる
            last_time = now_time
            last_dist = distance
        # r.sleep()
        if distance >= 1.0:                                                 # 距離(distance)が1[m]を越えたら、、、
            vel.linear.x = 0.10                                                  # ロボットから見て並進前方方向に0.0[m/s](停止)の速度を変数に代入                                                              # このwhile文を抜け出す
        if distance >= 2.0:                                                 # 距離(distance)が1[m]を越えたら、、、
            vel.linear.x = 0.0                                                  # ロボットから見て並進前方方向に0.0[m/s](停止)の速度を変数に代入
            break              

    # pub.publish(vel)                                                            # 17行目で宣言したとおり、速度を発行している
    # rospy.spin()                                                                # 36行目までのプログラムが起動し役目を終えるまでプログラムが終了しないようにしている


if __name__ == '__main__':
    rospy.init_node('move_1meter_develop')                                          # このプログラムを任意のノード名で登録する
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size= 10 ) # ロボットの速度を求めているTopic('/mobile_base/commands/velocity')にTwist型のメッセージをPublishすることを宣言
    sub = rospy.Subscriber('/odom', Odometry, callback_odom)                        # ロボットの位置を示しているTopic('/odom')にOdometry型のメッセージを取得することを宣言
    main(0.05, 0.0, 1.0)
    main(0.10, 0.0, 1.0)
    zero_vel = Twist()                           # 速度を送るための変数をTwist型で宣言
    zero_vel.linear.x = 0.0                      # ロボットから見て並進前方方向に0.0[m/s]の速度を変数に代入
    zero_vel.linear.y = 0.0                      # ロボットから見て並進前方方向に0.0[m/s]の速度を変数に代入
    zero_vel.angular.z = 0.0                     # ロボットから見て並進前方方向に0.0[rad/s]の速度を変数に代入
    pub.publish(zero_vel)
    rospy.spin()  


##  roslaunch sobit_pro_bringup minimal.launch enable_mb:=true enable_arm:=false enable_head:=false
##  rosrun navigation_stack test_wheel.py