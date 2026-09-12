import pyhula
import time
import math

#着陸を開始するマーカーとの距離(cm)
touchdown_qr_distance = 100




api = pyhula.UserApi()
if not api.connect():
    print("connect error")
else:
    print("connection to station by wifi")

print(f"battery={api.get_battery()}")

# ストリームを有効化（この順番が重要）
api.Plane_cmd_swith_rtp(0)
api.single_fly_flip_rtp()

# SPS/PPSが届くまで待機
time.sleep(3)

api.single_fly_takeoff()

print("映像取得開始...")

while True:
    api.single_fly_fowrward(15)

    array = api.single_fly_Anticipatory_recognition(0)

    distance_x = array['x']
    distance_y = array['y']
    distance_z = array['z']
    qr_result = array['result']
    distance_xy = math.sqrt(distance_x**2 + distance_y**2)
    distance_xyz = math.sqrt(distance_x**2 + distance_y**2 + distance_z**2)

    print(f"x距離:{distance_x}cm y距離:{distance_y}cm z距離:{distance_z}cm 水平距離:{distance_xy}cm 立体距離:{distance_xyz}cm 結果:{qr_result}")

    if qr_result and distance_y < touchdown_qr_distance:
        print("マーカー発見")
        break

api.single_fly_touchdown()

# パラメータ
# x：座標x（cm）（機体の左右、右が正）y：座標y（cm）（機体の前後、前が正）
# z：座標z（cm）（機体の上下、上が正）
# らしい

