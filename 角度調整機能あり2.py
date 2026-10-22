import pyhula
import time
import math

# ==================================================
# 座標の向き(ドローン基準)
#   x : 右が正
#   y : 前が正
#   z : 上が正
# camera_angle(カメラの角度)は「水平方向から下向きに何度か」を表す
#   0  → 真正面
#   90 → 真下
# ==================================================


# ==================================================
# 設定値
# ==================================================

# 移動スピード(cm/s)
moving_speed = 50


# ==================================================
# 関数
# ==================================================

# カメラの角度を変更する関数
# 変更後の角度をそのまま返すので「camera_angle = set_camera_angle(90)」のように使う
def set_camera_angle(angle):
    api.Plane_cmd_camera_angle(1, angle)
    return angle


# 相対位置をカメラ基準からドローン基準に直す関数
def camera_to_drone(camera_relative_x, camera_relative_y, camera_relative_z, camera_angle):
    angle_rad = math.radians(camera_angle)

    drone_relative_x = camera_relative_x
    drone_relative_y = camera_relative_y * math.cos(angle_rad) + camera_relative_z * math.sin(angle_rad)
    drone_relative_z = camera_relative_z * math.cos(angle_rad) - camera_relative_y * math.sin(angle_rad)
    return drone_relative_x, drone_relative_y, drone_relative_z


# QRコードに近づく関数
# relative_x, relative_y : ドローン基準でのQRの相対位置(cm)
# scale                  : 移動距離にかける倍率(1なら相対位置の分だけそのまま動く)
def qr_get_closer(relative_x, relative_y, scale):
    # 左右の移動
    if relative_x > 0:
        api.single_fly_right(int(relative_x * scale), moving_speed)
    if relative_x < 0:
        api.single_fly_left(int(abs(relative_x) * scale), moving_speed)

    # 前後の移動
    if relative_y > 0:
        api.single_fly_forward(int(relative_y * scale), moving_speed)
    if relative_y < 0:
        api.single_fly_back(int(abs(relative_y) * scale), moving_speed)


# QRを読み取ってから近づく関数
# 返り値は辞書で、QRの相対位置は「移動後」に読み取り直したもの
#   見つかった場合   : {"result": 結果, "relative_x": x, "relative_y": y, "relative_z": z}
#   見つからない場合 : {"result": 結果}
def auto_get_closer(scale, qr_number, camera_angle):
    array = api.single_fly_Anticipatory_recognition(qr_number)

    # QRが見つからなかった場合
    if not array["result"]:
        print("QR NOT FOUND")
        return {"result": array["result"]}

    relative_x, relative_y, relative_z = camera_to_drone(array["x"], array["y"], array["z"], camera_angle)

    # 既に十分近い(左右前後とも10cm未満)なら動かない
    if abs(relative_x) < 10 and abs(relative_y) < 10:
        print("既に十分近い位置にいます")
        return {"result": array["result"],
                "relative_x": relative_x,
                "relative_y": relative_y,
                "relative_z": relative_z}

    # 近づいてから、もう一度QRを読み取る
    print(f"ドローン基準でQRの相対位置({relative_x:.2f},{relative_y:.2f},{relative_z:.2f})に近づきます")
    qr_get_closer(relative_x, relative_y, scale)
    array = api.single_fly_Anticipatory_recognition(qr_number)
    relative_x, relative_y, relative_z = camera_to_drone(array["x"], array["y"], array["z"], camera_angle)

    return {"result": array["result"],
            "relative_x": relative_x,
            "relative_y": relative_y,
            "relative_z": relative_z}


# QRコードが見つかるまで探す関数
# qr_number       : 探すQRの番号
# search_mode     : 一度も見つけていないときの動き方
#                     "forward" → 25cm前進
#                     "turn"    → 右に30度回転
# qr_found_before : 以前にQRを一度でも見つけたかどうか(True / False)
# camera_angle    : 今のカメラの角度
# turn_angle      : これまでに右へ回転した累積角度
# 返り値 : (QRの読み取り結果, qr_found_before, camera_angle, turn_angle)
#          カメラの角度と累積回転角度が途中で変わることがあるので、それらも返す
def search_qr(qr_number, search_mode, qr_found_before, camera_angle, turn_angle):
    while True:

        # QRが見つかった場合
        array = api.single_fly_Anticipatory_recognition(qr_number)
        if array["result"]:
            print("QR発見")
            qr_found_before = True
            return array, qr_found_before, camera_angle, turn_angle

        if qr_found_before:
            # 一度発見したのに見失った場合は最大2回まで下がる
            found_again = False
            back_count = 0
            while back_count < 2:
                back_count += 1
                print("見失ったから後進")
                api.single_fly_back(40)
                camera_angle = set_camera_angle(60)
                array = api.single_fly_Anticipatory_recognition(qr_number)
                if array["result"]:
                    print("再発見")
                    found_again = True
                    break

            if found_again:
                return array, qr_found_before, camera_angle, turn_angle

            # 2回下がっても見つからなかったら、一度も見つけていない状態に戻す
            # (カメラの角度は60度のまま探索を続ける)
            print("後進したが見つからないのでリセット")
            qr_found_before = False

        else:
            # まだ一度も見つけていなくて発見できない場合
            if search_mode == "forward":
                api.single_fly_forward(25)
                print("QRないから前進")
            if search_mode == "turn":
                api.single_fly_turnright(30)
                turn_angle += 30
                print(f"QRないから右30度回転(累積{turn_angle}度)")


# 真上からQRに近づきながら高度を下げ、着地する関数
# turn_angle : これまでに右へ回転した累積角度
# 返り値 : 着地できたら True、途中でQRを見失ったら False
def land_on_qr(qr_number, camera_angle, turn_angle):
    # 高度を下げた直後かどうか
    low_height = False

    while True:
        auto_closer_return = auto_get_closer(1, qr_number, camera_angle)
        print(auto_closer_return)

        if auto_closer_return["result"]:

            # 左右前後とも12cm未満なら真上にいるとみなして着地
            if abs(auto_closer_return["relative_x"]) < 12 and abs(auto_closer_return["relative_y"]) < 12:
                # 斜めのまま着地しないよう、累積角度を90で割った余りだけ左に戻す
                remainder = turn_angle % 90
                if remainder != 0:
                    print(f"向きを揃えるため左に{remainder}度回転")
                    api.single_fly_turnleft(remainder)
                print("真上なので着地します")
                api.single_fly_touchdown()
                return True

            # 左右前後とも30cm未満なら高度を下げる
            if abs(auto_closer_return["relative_x"]) < 30 and abs(auto_closer_return["relative_y"]) < 30:
                print("近いので高度を下げます")
                api.single_fly_down(35)
                low_height = True

        else:
            # 見失った場合は高度を調整してから探し直す
            print("見逃した")
            if low_height:
                low_height = False
                api.single_fly_up(15)
            else:
                api.single_fly_down(20)
            return False


# 離陸してから指定したQRを探し、そこへ飛んで行って着陸する関数
# 「遠くから探すQR」と「真下を向いてから着陸するQR」の番号を別々に指定できる
#   search_qr_number  : 遠くから探して、近づく目印にするQRの番号
#   landing_qr_number : 真下を向いたあと、位置を合わせて着陸するQRの番号
#   ※ 同じQRに着陸したいときは、2つとも同じ番号にする
# search_mode  : QRが見つからないときの動き方("forward" または "turn")
# camera_angle : 離陸前に設定しておいたカメラの角度
def takeoff_and_land_on_qr(search_qr_number, landing_qr_number, search_mode, camera_angle):
    qr_found_before = False
    # 探索中に右へ回転した累積角度
    turn_angle = 0

    # 離陸
    api.single_fly_takeoff()

    # 着地できるまで「探す → 近づく → 降りる」を繰り返す
    while True:
        # 目印のQRを探す(search_qr_number を使う)
        array, qr_found_before, camera_angle, turn_angle = search_qr(search_qr_number, search_mode, qr_found_before, camera_angle, turn_angle)

        # QRに向かって行って下を向く
        relative_x, relative_y, relative_z = camera_to_drone(array["x"], array["y"], array["z"], camera_angle)
        print(f"ドローン基準でのQRの相対位置(x,y,z): {relative_x:.2f}, {relative_y:.2f}, {relative_z:.2f}")
        qr_get_closer(relative_x, relative_y, 0.9)
        print("接近したから真下を向く")
        camera_angle = set_camera_angle(90)
        time.sleep(1)
        api.single_fly_up(20)

        # 着陸用のQRに真上から近づいて着地する(landing_qr_number を使う)
        # 見失ったら、もう一度「目印のQRを探す」ところからやり直す
        landed = land_on_qr(landing_qr_number, camera_angle, turn_angle)
        if landed:
            print(f"{landing_qr_number}番に着陸しました")
            return


# ==================================================
# メイン処理
# ==================================================

# API読み込みコピペ
api = pyhula.UserApi()
if not api.connect():
    print("connect error")
else:
    print("connection to station by wifi")

print(f"battery={api.get_battery()}")

# ストリームを有効化
api.Plane_cmd_swith_rtp(0)
api.single_fly_flip_rtp()
print("映像取得開始...")

# takeoff_and_land_on_qr(探すQRの番号, 着陸するQRの番号, 探し方, カメラの角度)
#   例:takeoff_and_land_on_qr(3, 4, "turn", camera_angle)
#       → 3番のQRを目印に近づき、真下を向いたら4番のQRに着陸する

# ---------- 0番のQRに着陸 ----------
camera_angle = set_camera_angle(45)

# SPS/PPSが届くまで待機
time.sleep(3)

takeoff_and_land_on_qr(0, 0, "forward", camera_angle)

# ---------- 1番のQRに着陸 ----------
# 10秒待機
time.sleep(10)
camera_angle = set_camera_angle(45)
takeoff_and_land_on_qr(1, 1, "turn", camera_angle)

# ---------- 2番のQRに着陸 ----------
# 10秒待機
time.sleep(10)
camera_angle = set_camera_angle(45)
takeoff_and_land_on_qr(2, 2, "turn", camera_angle)
