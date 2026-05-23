# app.py

from flask import Flask, request, jsonify, render_template
import firebase_admin
from firebase_admin import credentials, db
from datetime import datetime
import os
import time

app = Flask(__name__)

# =========================================================
# Firebase Realtime Database 연결 설정
# =========================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# app.py 기준:
# project_folder/
#   app.py
#   config/
#     serviceAccountKey.json
KEY_PATH = os.path.join(BASE_DIR, "config", "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json")

DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app"

if not os.path.exists(KEY_PATH):
    raise FileNotFoundError(
        f"Firebase 인증키 파일을 찾을 수 없습니다: {KEY_PATH}"
    )

if not firebase_admin._apps:
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred, {
        "databaseURL": DATABASE_URL
    })


# =========================================================
# 메인 웹 페이지
# =========================================================

@app.route("/")
def index():
    return render_template("/home/yoon/cobot_ws/src/cobot1/web_ui/index_03.html")


# =========================================================
# 웹 command 이름을 service_bridge action 이름으로 변환
# =========================================================

def command_to_action(command):
    """
    웹에서 보내는 command 이름과
    service_bridge.py가 처리하는 action 이름을 맞춰주는 함수
    """

    command_map = {
        "movej": "move_joint",
        "movel": "move_line",
        "home": "move_home",
        "change_speed": "change_speed",

        # 아래 명령들은 현재 service_bridge에서 로그/상태용으로만 처리 가능
        "tool_on": "tool_on",
        "tool_off": "tool_off",
        "tcp_check": "tcp_check",
        "tool_check": "tool_check"
    }

    return command_map.get(command)


# =========================================================
# Firebase Realtime Database에 로봇 명령 저장
# =========================================================

def save_robot_command(command, pos=None, vel=60, acc=60, speed=100):
    """
    service_bridge.py가 감시하는 Firebase 경로:
        commands/dsr01

    저장 형식:
        {
            "action": "move_joint",
            "pos": [0, 0, 90, 0, 90, 0],
            "vel": 60,
            "acc": 60,
            "status": "pending",
            "timestamp": ...
        }
    """

    action = command_to_action(command)

    if action is None:
        raise ValueError(f"지원하지 않는 command입니다: {command}")

    command_data = {
        "action": action,
        "status": "pending",
        "created_at": datetime.now().isoformat(),
        "timestamp": int(time.time() * 1000)
    }

    if action in ["move_joint", "move_line"]:
        command_data["pos"] = pos
        command_data["vel"] = vel
        command_data["acc"] = acc

    elif action == "move_home":
        command_data["vel"] = vel
        command_data["acc"] = acc

    elif action == "change_speed":
        command_data["speed"] = speed

    else:
        # tool_on, tool_off, tcp_check, tool_check
        command_data["vel"] = vel
        command_data["acc"] = acc

    # 중요:
    # push()를 쓰지 않고 set()을 사용한다.
    # service_bridge.py는 commands/dsr01 바로 아래의 action을 읽기 때문이다.
    ref = db.reference("commands/dsr01")
    ref.set(command_data)

    return command_data


# =========================================================
# 웹에서 명령 받는 API
# =========================================================

@app.route("/api/robot", methods=["POST"])
def robot_command():
    data = request.get_json()

    if data is None:
        return jsonify({
            "success": False,
            "message": "JSON 데이터가 없습니다."
        }), 400

    command = data.get("command")
    pos = data.get("pos")
    vel = data.get("vel", 60)
    acc = data.get("acc", 60)
    speed = data.get("speed", 100)

    try:
        # ---------------------------------------------------------
        # MoveJ / MoveL
        # ---------------------------------------------------------
        if command in ["movej", "movel"]:
            if pos is None:
                return jsonify({
                    "success": False,
                    "message": "movej/movel 명령에는 pos 값이 필요합니다."
                }), 400

            if not isinstance(pos, list) or len(pos) != 6:
                return jsonify({
                    "success": False,
                    "message": "pos는 6개의 숫자 리스트여야 합니다."
                }), 400

            saved_command = save_robot_command(
                command=command,
                pos=pos,
                vel=vel,
                acc=acc
            )

        # ---------------------------------------------------------
        # Home
        # ---------------------------------------------------------
        elif command == "home":
            saved_command = save_robot_command(
                command=command,
                vel=vel,
                acc=acc
            )

        # ---------------------------------------------------------
        # 속도 변경
        # ---------------------------------------------------------
        elif command == "change_speed":
            saved_command = save_robot_command(
                command=command,
                speed=speed
            )

        # ---------------------------------------------------------
        # Tool / TCP 체크 명령
        # ---------------------------------------------------------
        elif command in ["tool_on", "tool_off", "tcp_check", "tool_check"]:
            saved_command = save_robot_command(
                command=command,
                vel=vel,
                acc=acc
            )

        else:
            return jsonify({
                "success": False,
                "message": f"알 수 없는 명령입니다: {command}"
            }), 400

        return jsonify({
            "success": True,
            "message": f"{command} 명령이 Firebase commands/dsr01에 저장되었습니다.",
            "data": saved_command
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        }), 500


# =========================================================
# 현재 로봇 상태 가져오기
# =========================================================

@app.route("/api/status", methods=["GET"])
def get_status():
    try:
        ref = db.reference("robot_status/current")
        status_data = ref.get()

        if status_data is None:
            status_data = {
                "tcp_ready": False,
                "tool_ready": False,
                "gripper_model": "unknown",
                "tool_weight": 0.0,
                "tcp_pos": [0, 0, 0, 0, 0, 0],
                "joint_pos": [0, 0, 0, 0, 0, 0],
                "message": "아직 robot_status/current 데이터가 없습니다."
            }

        return jsonify({
            "success": True,
            "status": status_data
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        }), 500


# =========================================================
# 서버 실행
# =========================================================

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True
    )