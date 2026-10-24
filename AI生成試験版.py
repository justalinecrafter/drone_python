import pyhula
import time
import math
import cv2
import numpy as np

# ==================================================
# camera_angle(カメラの角度)は「水平方向から下向きに何度か」を表す
#   0  → 真正面
#   90 → 真下
# ==================================================

# ==================================================
# 設定値
# ==================================================

# 移動スピード(cm/s)
moving_speed = 50

# ---------- 色紙の認識に使う設定値 ----------

# カメラの水平方向の視野角(度)
#   測り方:カメラを真下に向けて高さ h(cm) から床を映し、
#           画面の左端から右端までの長さ L(cm) を測る
#           視野角 = 2 × atan((L / 2) / h) を度に直した値
camera_hfov = 60

# カメラ画像の色の並び
#   pyhula の画像は RGB の順番なので True
#   実機で赤と青が入れ替わって認識される場合は False にする
image_is_rgb = True

# 画像を取得する前に待つ時間(秒)
color_wait_time = 0.5

# 色紙とみなす最小の大きさ(ピクセル数)
color_min_area = 500

# 認識する色の範囲(HSV)
#   [H(色相), S(鮮やかさ), V(明るさ)] の下限と上限
#   OpenCV では H は 0〜179、S と V は 0〜255
#   部屋の明るさや紙の色に合わせて調整すること
# 赤は H が 0 付近と 179 付近の両端にあるので、範囲を2つ使う
red_lower_1 = [0, 120, 70]
red_upper_1 = [10, 255, 255]
red_lower_2 = [170, 120, 70]
red_upper_2 = [179, 255, 255]
# 緑
green_lower = [40, 80, 50]
green_upper = [85, 255, 255]
# 青
blue_lower = [100, 120, 50]
blue_upper = [130, 255, 255]

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
#   search_qr_number  : 遠くから探して、近づく目印にするQRの番号
#   landing_qr_number : 真下を向いたあと、位置を合わせて着陸するQRの番号
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
# 色紙(R/G/B の単色の紙)用の関数
# QR用の関数と同じ流れで動くように作ってある
#   QR                       色紙
#   auto_get_closer        → color_auto_get_closer
#   search_qr              → search_color
#   land_on_qr             → land_on_color
#   takeoff_and_land_on_qr → takeoff_and_land_on_color
# 色の指定は "R"(赤)、"G"(緑)、"B"(青) の文字で行う
# ==================================================

# 画像の上の位置(ピクセル)から、床の上の位置(ドローン基準, cm)を求める関数
# 床が平らだと考えて、「カメラから物体へ向かう線」が床とぶつかる点を計算する
# pixel_x, pixel_y : 物体の中心の位置(画像の左上が 0, 0)
# image_width, image_height : 画像の横幅と縦幅(ピクセル)
# camera_angle : カメラの角度(水平から下向きに何度か)
# height       : ドローンの高度(cm)
# 返り値 : (x, y)  x は右が正、y は前が正(cm)
#          床に届かない向き(地平線より上)のときは None
def pixel_to_floor(pixel_x, pixel_y, image_width, image_height, camera_angle, height):
    # 視野角から、レンズの焦点距離をピクセル単位で求める
    focal = (image_width / 2) / math.tan(math.radians(camera_hfov) / 2)

    # 画面の中心からのずれを「向き」に直す
    #   right_ratio : 右にどれだけずれているか(画面の右が正)
    #   up_ratio    : 上にどれだけずれているか(画面の上が正)
    right_ratio = (pixel_x - image_width / 2) / focal
    up_ratio = (image_height / 2 - pixel_y) / focal

    # カメラの傾きを考えて、線がどれだけ下に向かっているかを求める
    angle_rad = math.radians(camera_angle)
    down_ratio = math.sin(angle_rad) - up_ratio * math.cos(angle_rad)

    # 下に向かっていない線は床とぶつからない
    if down_ratio <= 0:
        return None

    # 床とぶつかる点を求める
    distance = height / down_ratio
    x = right_ratio * distance
    y = (math.cos(angle_rad) + up_ratio * math.sin(angle_rad)) * distance
    return x, y


# HSV画像から、指定した色の部分だけを白、それ以外を黒にした画像(マスク)を作る関数
# color_name : "R"、"G"、"B" のどれか
# 返り値 : マスク画像(色の名前が間違っているときは None)
def make_color_mask(hsv, color_name):
    if color_name == "R":
        # 赤は範囲が2つあるので、それぞれのマスクを作って合わせる
        mask_1 = cv2.inRange(hsv, np.array(red_lower_1), np.array(red_upper_1))
        mask_2 = cv2.inRange(hsv, np.array(red_lower_2), np.array(red_upper_2))
        return cv2.bitwise_or(mask_1, mask_2)
    if color_name == "G":
        return cv2.inRange(hsv, np.array(green_lower), np.array(green_upper))
    if color_name == "B":
        return cv2.inRange(hsv, np.array(blue_lower), np.array(blue_upper))

    print(f"色の指定が間違っています: {color_name}(\"R\", \"G\", \"B\" のどれかにする)")
    return None


# 色紙を認識する関数(QRの api.single_fly_Anticipatory_recognition の代わり)
# color_name   : "R"、"G"、"B" のどれか
# camera_angle : 今のカメラの角度
# 返り値は辞書
#   見つかった場合   : {"result": True, "x": x, "y": y, "z": z, "area": 面積}
#   見つからない場合 : {"result": False}
#   ※ QRと違い、x, y, z は最初から「ドローン基準」の位置(cm)
#      そのため camera_to_drone で変換する必要はない
def recognize_color(color_name, camera_angle):
    # 映像が追いつくまで少し待ってから、カメラ画像を取得する
    time.sleep(color_wait_time)
    frame = api.get_image_array()
    if frame is None:
        print("カメラ画像が取得できませんでした")
        return {"result": False}

    # OpenCV で扱えるように、色の並びを BGR にしてから HSV に変換する
    if image_is_rgb:
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        bgr = frame
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # 指定した色の部分だけを取り出す
    mask = make_color_mask(hsv, color_name)
    if mask is None:
        return {"result": False}

    # 色のかたまり(輪郭)を探す
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return {"result": False}

    # 一番大きいかたまりを色紙とみなす
    biggest = contours[0]
    for contour in contours:
        if cv2.contourArea(contour) > cv2.contourArea(biggest):
            biggest = contour

    # 小さすぎるかたまりはノイズとして無視する
    area = cv2.contourArea(biggest)
    if area < color_min_area:
        return {"result": False}

    # かたまりの中心(重心)の位置を求める
    moments = cv2.moments(biggest)
    if moments["m00"] == 0:
        return {"result": False}
    center_x = moments["m10"] / moments["m00"]
    center_y = moments["m01"] / moments["m00"]

    # ドローンの高度を取得する(ToFセンサー。単位は cm と思われる)
    height = api.get_plane_distance()
    if height is None or height <= 0:
        print("高度が取得できませんでした")
        return {"result": False}

    # 画像の上の位置を、床の上の位置(ドローン基準)に直す
    image_height = frame.shape[0]
    image_width = frame.shape[1]
    position = pixel_to_floor(center_x, center_y, image_width, image_height, camera_angle, height)
    if position is None:
        return {"result": False}

    x, y = position
    return {"result": True, "x": x, "y": y, "z": -height, "area": area}


# 色紙に近づく関数(QRの qr_get_closer と同じ動き)
# relative_x, relative_y : ドローン基準での色紙の相対位置(cm)
# scale                  : 移動距離にかける倍率(1なら相対位置の分だけそのまま動く)
def color_get_closer(relative_x, relative_y, scale):
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


# 色紙を認識してから近づく関数(QRの auto_get_closer と同じ動き)
# 返り値は辞書で、色紙の相対位置は「移動後」に認識し直したもの
#   見つかった場合   : {"result": True, "relative_x": x, "relative_y": y, "relative_z": z}
#   見つからない場合 : {"result": False}
def color_auto_get_closer(scale, color_name, camera_angle):
    array = recognize_color(color_name, camera_angle)

    # 色紙が見つからなかった場合
    if not array["result"]:
        print("COLOR NOT FOUND")
        return {"result": False}

    relative_x = array["x"]
    relative_y = array["y"]
    relative_z = array["z"]

    # 既に十分近い(左右前後とも10cm未満)なら動かない
    if abs(relative_x) < 10 and abs(relative_y) < 10:
        print("既に十分近い位置にいます")
        return {"result": True,
                "relative_x": relative_x,
                "relative_y": relative_y,
                "relative_z": relative_z}

    # 近づいてから、もう一度認識する
    print(f"ドローン基準で色紙の相対位置({relative_x:.2f},{relative_y:.2f},{relative_z:.2f})に近づきます")
    color_get_closer(relative_x, relative_y, scale)
    array = recognize_color(color_name, camera_angle)

    # 近づいたあとに見失った場合
    if not array["result"]:
        return {"result": False}

    return {"result": True,
            "relative_x": array["x"],
            "relative_y": array["y"],
            "relative_z": array["z"]}


# 色紙が見つかるまで探す関数(QRの search_qr と同じ動き)
# color_name         : 探す色("R"、"G"、"B")
# search_mode        : 一度も見つけていないときの動き方
#                        "forward" → 25cm前進
#                        "turn"    → 右に30度回転
# color_found_before : 以前に色紙を一度でも見つけたかどうか(True / False)
# camera_angle       : 今のカメラの角度
# turn_angle         : これまでに右へ回転した累積角度
# 返り値 : (認識結果, color_found_before, camera_angle, turn_angle)
def search_color(color_name, search_mode, color_found_before, camera_angle, turn_angle):
    while True:

        # 色紙が見つかった場合
        array = recognize_color(color_name, camera_angle)
        if array["result"]:
            print(f"色紙({color_name})発見")
            color_found_before = True
            return array, color_found_before, camera_angle, turn_angle

        if color_found_before:
            # 一度発見したのに見失った場合は最大2回まで下がる
            found_again = False
            back_count = 0
            while back_count < 2:
                back_count += 1
                print("見失ったから後進")
                api.single_fly_back(40)
                camera_angle = set_camera_angle(60)
                array = recognize_color(color_name, camera_angle)
                if array["result"]:
                    print("再発見")
                    found_again = True
                    break

            if found_again:
                return array, color_found_before, camera_angle, turn_angle

            # 2回下がっても見つからなかったら、一度も見つけていない状態に戻す
            print("後進したが見つからないのでリセット")
            color_found_before = False

        else:
            # まだ一度も見つけていなくて発見できない場合
            if search_mode == "forward":
                api.single_fly_forward(25)
                print("色紙ないから前進")
            if search_mode == "turn":
                api.single_fly_turnright(30)
                turn_angle += 30
                print(f"色紙ないから右30度回転(累積{turn_angle}度)")


# 真上から色紙に近づきながら高度を下げ、着地する関数(QRの land_on_qr と同じ動き)
# turn_angle : これまでに右へ回転した累積角度
# 返り値 : 着地できたら True、途中で色紙を見失ったら False
def land_on_color(color_name, camera_angle, turn_angle):
    # 高度を下げた直後かどうか
    low_height = False

    while True:
        auto_closer_return = color_auto_get_closer(1, color_name, camera_angle)
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


# 離陸してから指定した色の色紙を探し、その上に着陸する関数
# (QRの takeoff_and_land_on_qr と同じ動き)
# color_name   : 着陸したい色紙の色("R"、"G"、"B")
# search_mode  : 色紙が見つからないときの動き方("forward" または "turn")
# camera_angle : 離陸前に設定しておいたカメラの角度
def takeoff_and_land_on_color(color_name, search_mode, camera_angle):
    color_found_before = False
    # 探索中に右へ回転した累積角度
    turn_angle = 0

    # 離陸
    api.single_fly_takeoff()

    # 着地できるまで「探す → 近づく → 降りる」を繰り返す
    while True:
        # 色紙を探す
        array, color_found_before, camera_angle, turn_angle = search_color(color_name, search_mode, color_found_before, camera_angle, turn_angle)

        # 色紙に向かって行って下を向く
        # (色紙の位置は最初からドローン基準なので、そのまま使う)
        relative_x = array["x"]
        relative_y = array["y"]
        relative_z = array["z"]
        print(f"ドローン基準での色紙の相対位置(x,y,z): {relative_x:.2f}, {relative_y:.2f}, {relative_z:.2f}")
        color_get_closer(relative_x, relative_y, 0.9)
        print("接近したから真下を向く")
        camera_angle = set_camera_angle(90)
        time.sleep(1)
        api.single_fly_up(20)

        # 真上から近づいて着地する(見失ったらもう一度探すところから)
        landed = land_on_color(color_name, camera_angle, turn_angle)
        if landed:
            print(f"{color_name}の色紙に着陸しました")
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

# # ---------- 0番のQRに着陸 ----------
# camera_angle = set_camera_angle(45)

# # SPS/PPSが届くまで待機
# time.sleep(3)

# takeoff_and_land_on_qr(0, 0, "forward", camera_angle)

# # ---------- 1番のQRに着陸 ----------
# # 10秒待機
# time.sleep(10)
# camera_angle = set_camera_angle(45)
# takeoff_and_land_on_qr(1, 1, "turn", camera_angle)

# # ---------- 2番のQRに着陸 ----------
# # 10秒待機
# time.sleep(10)
# camera_angle = set_camera_angle(45)
# takeoff_and_land_on_qr(2, 2, "turn", camera_angle)

# ---------- 色紙に着陸する場合の例 ----------
# # 使うときは、行の先頭の「# 」を消す
# 色は "R"(赤)、"G"(緑)、"B"(青) で指定する

time.sleep(10)
camera_angle = set_camera_angle(45)
takeoff_and_land_on_color("R", "turn", camera_angle)
