"""
ros_firebase_bridge.py
DSR M0609 — ROS2 ↔ Firebase Realtime DB 브리지

Firebase DB 경로:
  robot/state    ROS → 웹: 로봇 상태
  robot/stats    ROS → 웹: 검사 통계
  robot/defect   ROS → 웹: 불량 데이터 (팝업 트리거)
  robot/joints   ROS → 웹: 현재 조인트 각도 (J1~J6, deg)
  robot/task     ROS → 웹: 현재 태스크 좌표 (X/Y/Z mm, RX/RY/RZ deg)
  robot/command  웹 → ROS: 검사 시작/정지
  robot/control  웹 → ROS: 조인트·태스크·그리퍼·비상정지 명령

의존성:
    pip install firebase-admin
    (rclpy, dsr_msgs는 ROS2 환경에서 자동 사용 가능)
"""

import json
import math
import base64
import os
import threading
import time
import logging
from datetime import datetime

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s %(message)s')
log = logging.getLogger(__name__)

# ══════════════════════════════════════════════════════
# ★ Firebase 설정 ★
# ══════════════════════════════════════════════════════
FIREBASE_CREDENTIAL_PATH = "serviceAccountKey.json"
FIREBASE_STORAGE_BUCKET  = "rokey-d-2-4c32a.firebasestorage.app"
FIREBASE_DATABASE_URL    = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app"

# ══════════════════════════════════════════════════════
# ROS2 토픽 / 서비스
# ══════════════════════════════════════════════════════
TOPIC_ROBOT_STATUS    = "/dsr01/progress_status"
TOPIC_INSP_RESULT     = "/dsr01/inspection_result"

# DSR 제어 서비스
SRV_MOVE_JOINT   = "/dsr01/motion/move_joint"
SRV_MOVE_TASK    = "/dsr01/motion/move_line"
SRV_GRIPPER_CTRL = "/dsr01/io/set_ctrl_box_digital_output"
SRV_DRL_STOP     = "/dsr01/drl/drl_stop"
SRV_DRL_START    = "/dsr01/drl/drl_start"
SRV_SERVO_OFF    = "/dsr01/system/servo_off"
SRV_ROBOT_CTRL   = "/dsr01/system/set_robot_control"
SRV_PAUSE             = "/dsr01/motion/move_pause"
SRV_RESUME            = "/dsr01/motion/move_resume"
SRV_SET_SAFETY_MODE   = "/dsr01/system/set_safety_mode"    # 안전 모드 전환

# SetSafetyMode 상수 (enum.SAFETY_MODE / enum.SAFETY_MODE_EVENT)
SAFETY_MODE_MANUAL      = 0   # 수동 모드
SAFETY_MODE_AUTONOMOUS  = 1   # 자율(정상) 모드
SAFETY_MODE_RECOVERY    = 2   # 복구 모드 — 보호정지 중 저속 제어 가능
SAFETY_MODE_EVENT_ENTER = 0   # safety_event: 모드 진입
SAFETY_MODE_EVENT_MOVE  = 1   # safety_event: 복구 모드 내 이동
SAFETY_MODE_EVENT_STOP  = 2   # safety_event: 복구 모드 내 정지

MOVE_REFERENCE_BASE = 0; MOVE_REFERENCE_TOOL = 1

# Firebase Realtime DB 경로
DB_STATE   = "robot/state"
DB_STATS   = "robot/stats"
DB_DEFECT  = "robot/defect"
DB_JOINTS  = "robot/joints"    # 현재 조인트 각도
DB_TASK    = "robot/task"      # 현재 태스크(직교) 좌표 ← 신규
DB_COMMAND = "robot/command"
DB_CONTROL      = "robot/control"
DB_ROBOT_STATE  = "robot/robot_state"  # 로봇 상태 코드 (0~15)
DB_RECOVERY     = "robot/recovery_mode"  # 복구 모드 상태 웹 표시용

DEFECTS_COLLECTION = "defects"

# ══════════════════════════════════════════════════════
# Firebase / ROS 초기화
# ══════════════════════════════════════════════════════
try:
    import firebase_admin
    from firebase_admin import credentials, firestore, storage as fb_storage, db as rtdb
    FIREBASE_AVAILABLE = True
    log.info("Firebase Admin SDK 로드됨")
except ImportError:
    FIREBASE_AVAILABLE = False
    log.error("firebase-admin 미설치 → pip install firebase-admin")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
    log.info("rclpy 로드됨")
except ImportError:
    ROS_AVAILABLE = False
    log.error("rclpy 미설치 — ROS2 환경에서만 실행 가능합니다")

try:
    from dsr_msgs2.srv import MoveJoint, MoveLine, MovePause, MoveResume, DrlStop, DrlStart, SetRobotControl, Jog
    from dsr_msgs2.msg import RobotState
    from dsr_msgs2.action import MovejH2r, MovelH2r
    from rclpy.action import ActionClient
    DSR_MSGS_AVAILABLE = True
    log.info("dsr_msgs2 로드됨")
except ImportError:
    DSR_MSGS_AVAILABLE = False
    log.warning("dsr_msgs2 미설치 — 제어 서비스 토픽 폴백 사용")

DSR_ROBOT2_AVAILABLE = False  # 노드 생성 후 init_dsr_robot2()에서 설정
_dsr2_executor = None  # DSR_ROBOT2 전용 executor

db_client      = None
storage_bucket = None
rt_root        = None

def init_firebase():
    global db_client, storage_bucket, rt_root
    if not FIREBASE_AVAILABLE:
        log.error("Firebase 사용 불가"); return False
    if not os.path.exists(FIREBASE_CREDENTIAL_PATH):
        log.error(f"키 파일 없음: {FIREBASE_CREDENTIAL_PATH}"); return False
    try:
        cred = credentials.Certificate(FIREBASE_CREDENTIAL_PATH)
        firebase_admin.initialize_app(cred, {
            "storageBucket": FIREBASE_STORAGE_BUCKET,
            "databaseURL":   FIREBASE_DATABASE_URL,
        })
        db_client      = firestore.client()
        storage_bucket = fb_storage.bucket()
        rt_root        = rtdb.reference("/")
        log.info("Firebase 초기화 성공")
        return True
    except Exception as e:
        log.error(f"Firebase 초기화 실패: {e}"); return False

def init_dsr_robot2():
    """DSR_ROBOT2 전용 노드를 별도 스레드/executor로 실행 — 브리지 spin에 영향 없음"""
    global DSR_ROBOT2_AVAILABLE, _dsr2_executor
    def _run():
        global DSR_ROBOT2_AVAILABLE, _dsr2_executor
        try:
            import DR_init
            from rclpy.executors import SingleThreadedExecutor
            helper = rclpy.create_node("dsr_robot2_helper", namespace="dsr01")
            DR_init.__dsr__id    = "dsr01"
            DR_init.__dsr__model = "m0609"
            DR_init.__dsr__node  = helper
            from DSR_ROBOT2 import get_robot_state, drl_script_stop, DR_QSTOP_STO
            _dsr2_executor = SingleThreadedExecutor()
            _dsr2_executor.add_node(helper)
            DSR_ROBOT2_AVAILABLE = True
            log.info("DSR_ROBOT2 로드됨 (별도 executor)")
            _dsr2_executor.spin()  # 별도 스레드에서 blocking
        except Exception as e:
            DSR_ROBOT2_AVAILABLE = False
            log.warning(f"DSR_ROBOT2 로드 실패: {e}")
    threading.Thread(target=_run, daemon=True).start()
    time.sleep(1.5)  # 로드 완료 대기

# ══════════════════════════════════════════════════════
# Realtime DB 헬퍼
# ══════════════════════════════════════════════════════
def rt_set(path: str, value):
    if rt_root is None: return
    try: rt_root.child(path).set(value)
    except Exception as e: log.error(f"RT DB 쓰기 실패 ({path}): {e}")

def rt_get(path: str):
    if rt_root is None: return None
    try: return rt_root.child(path).get()
    except Exception as e: log.error(f"RT DB 읽기 실패 ({path}): {e}"); return None

# ══════════════════════════════════════════════════════
# 검사 세션
# ══════════════════════════════════════════════════════
class Session:
    def __init__(self):
        self.total = self.passed = self.failed = 0

session = Session()

def push_stats():
    rate = round(session.passed / session.total * 100, 1) if session.total > 0 else 0
    rt_set(DB_STATS, {"total": session.total, "passed": session.passed,
                      "failed": session.failed, "passRate": rate})



# ══════════════════════════════════════════════════════
# 로봇 제어 명령 실행
# ══════════════════════════════════════════════════════
def call_service(node, srv_type, srv_name, req, label):
    """서비스 클라이언트를 생성해 비동기 호출. 실패 시 에러 로그만 남김 (토픽 fallback 없음)."""
    try:
        cli = node.create_client(srv_type, srv_name)
        if cli.wait_for_service(timeout_sec=0.5):
            future = cli.call_async(req)
            future.add_done_callback(lambda f: log.info(f"✅ {label} 응답: {f.result()}"))
        else:
            log.error(f"❌ {label} 서비스 응답 없음: {srv_name}")
    except Exception as e:
        log.error(f"❌ {label} 서비스 호출 실패: {e}")



def call_robot_control(node, control_value, label):
    """SetRobotControl 서비스 — 서보 ON(3) / 보호정지 해제(2)"""
    try:
        from dsr_msgs2.srv import SetRobotControl
        cli = node.create_client(SetRobotControl, SRV_ROBOT_CTRL)
        if not cli.wait_for_service(timeout_sec=0.5):
            log.error(f"❌ {label} 서비스 응답 없음"); return False
        req = SetRobotControl.Request()
        req.robot_control = control_value
        future = cli.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > 5.0:
                log.error(f"❌ {label} 타임아웃"); return False
            time.sleep(0.01)
        res = future.result()
        log.info(f"✅ {label} 응답: success={res.success}")
        return res.success
    except Exception as e:
        log.error(f"❌ {label} 실패: {e}"); return False


def execute_control(node, ctrl: dict):
    t = ctrl.get("type", "")
    p = ctrl.get("payload", {})
    log.info(f"제어 명령: type={t}")

    if not node:
        log.error("노드 없음 — 제어 명령 무시"); return

    if t == "estop":
        log.warning("⚠ 비상정지")
        rt_set(DB_STATE, "estop")
        # 1. 스크립트 정지
        try:
            from DSR_ROBOT2 import drl_script_stop, DR_QSTOP_STO
            drl_script_stop(DR_QSTOP_STO)
            log.warning("비상정지: 스크립트 정지 완료")
        except Exception as e:
            log.error(f"drl_script_stop 실패: {e}")
        # 2. 서보 강제 OFF (스크립트 유무 상관없이)
        try:
            from dsr_msgs2.srv import ServoOff
            req = ServoOff.Request()
            req.stop_type = 0  # STOP_TYPE_QUICK_STO
            cli = node.create_client(ServoOff, SRV_SERVO_OFF)
            if cli.wait_for_service(timeout_sec=0.5):
                cli.call_async(req)
                log.warning("비상정지: 서보 OFF 명령 전송")
            else:
                log.error("ServoOff 서비스 응답 없음")
        except Exception as e:
            log.error(f"ServoOff 실패: {e}")

    elif t == "pause":
        log.info("일시정지")
        rt_set(DB_STATE, "pause")
        call_service(node, MovePause, SRV_PAUSE, MovePause.Request(), "일시정지(MovePause)")

    elif t == "resume":
        log.info("재개")
        rt_set(DB_STATE, "inspecting")
        call_service(node, MoveResume, SRV_RESUME, MoveResume.Request(), "재개(MoveResume)")

    elif t == "jog_h2r_start":
        # 조그 시작 — 버튼 누르는 동안 지속 이동
        # 복구 모드: DRL jog() 스크립트 실행 (일반 Jog 서비스는 복구 모드에서 막힘)
        # 일반 모드: Jog 서비스 사용
        axis     = int(p.get("axis", 0))
        velocity = float(p.get("velocity", 10.0))   # 부호 포함 (음수=반대방향)
        ref      = int(p.get("move_reference", MOVE_REFERENCE_BASE))
        recovery = p.get("recovery_mode", False)

        if recovery:
            # 복구 모드: movej_h2r / movel_h2r 액션 사용
            # axis 0~5 → 조인트, axis 6~11 → 태스크
            # 현재 위치에서 delta만큼 이동하는 목표를 보냄
            is_joint = (axis <= 5)
            cli = node._movej_h2r_cli if is_joint else node._movel_h2r_cli
            ActionType = MovejH2r if is_joint else MovelH2r

            if cli is None:
                log.error("[H2r] 액션 클라이언트 없음 — 복구 모드 진입 후 서보 ON 필요")
            elif cli.wait_for_server(timeout_sec=0.5):
                # 이전 목표 취소
                if node._recovery_goal_handle is not None:
                    try: node._recovery_goal_handle.cancel_goal_async()
                    except: pass
                    node._recovery_goal_handle = None

                # 현재 위치 읽기
                try:
                    from DSR_ROBOT2 import get_current_posj, get_current_posx
                    if is_joint:
                        cur = list(get_current_posj())
                        delta_idx = axis          # 조인트 축 인덱스
                    else:
                        cur = list(get_current_posx()[0])
                        delta_idx = axis - 6      # 태스크 축 인덱스
                    # 목표 = 현재 + delta (velocity 부호로 방향 결정)
                    DELTA = 5.0   # 한 번에 이동할 양 (°또는 mm)
                    cur[delta_idx] += DELTA if velocity > 0 else -DELTA
                    goal = ActionType.Goal()
                    goal.target_pos = [float(v) for v in cur]
                    spd = abs(velocity)
                    if is_joint:
                        goal.target_vel = [spd] * 6
                        goal.target_acc = [spd * 2] * 6
                    else:
                        goal.target_vel = [spd, spd]
                        goal.target_acc = [spd * 2, spd * 2]

                    def _goal_resp(future):
                        handle = future.result()
                        if handle.accepted:
                            node._recovery_goal_handle = handle
                            log.info(f"[H2r] 이동 시작 axis={axis} delta={'+'if velocity>0 else'-'}{DELTA}")
                        else:
                            log.warning("[H2r] 목표 거부됨")

                    fut = cli.send_goal_async(goal)
                    fut.add_done_callback(_goal_resp)
                except Exception as e:
                    log.error(f"[H2r] 목표 생성 실패: {e}")
            else:
                log.error("[H2r] 액션 서버 응답 없음")
        elif node._jog_cli is not None and node._jog_cli.wait_for_service(timeout_sec=0.5):
            req = Jog.Request()
            req.jog_axis       = axis
            req.move_reference = ref
            req.speed          = velocity
            fut = node._jog_cli.call_async(req)
            fut.add_done_callback(lambda f: log.info(f"[Jog] 시작 결과: {f.result()}"))
            log.info(f"[Jog] 시작 axis={axis} speed={velocity:.1f} ref={ref}")
        else:
            log.error("[Jog] 서비스 응답 없음")

    elif t == "jog_h2r_stop":
        # 조그 정지 — 버튼 뗄 때
        # 복구 모드: DRL 정지 (drl_stop)
        # 일반 모드: Jog 서비스 speed=0
        recovery = p.get("recovery_mode", False)

        if recovery:
            # 복구 모드: 진행 중인 H2r 액션 취소
            if node._recovery_goal_handle is not None:
                try:
                    node._recovery_goal_handle.cancel_goal_async()
                    log.info("[H2r] 이동 취소")
                except Exception as e:
                    log.error(f"[H2r] 취소 실패: {e}")
                node._recovery_goal_handle = None
        elif node._jog_cli is not None and node._jog_cli.wait_for_service(timeout_sec=0.5):
            req = Jog.Request()
            req.jog_axis       = 0
            req.move_reference = 0
            req.speed          = 0.0
            fut = node._jog_cli.call_async(req)
            fut.add_done_callback(lambda f: log.info(f"[Jog] 정지 결과: {f.result()}"))
            log.info("[Jog] 정지")
        else:
            log.error("[Jog] 정지 서비스 응답 없음")

    elif t == "joint":
        angles = [float(p.get(f"j{i}", 0.0)) for i in range(1, 7)]
        vel, acc = float(p.get("vel", 30.0)), float(p.get("acc", 60.0))
        req = MoveJoint.Request()
        req.pos = angles; req.vel = vel; req.acc = acc
        req.time = 0.0; req.radius = 0.0; req.mode = 0; req.blend_type = 0; req.sync_type = 0
        log.info(f"조인트 이동: {angles} vel={vel} acc={acc}")
        call_service(node, MoveJoint, SRV_MOVE_JOINT, req, "조인트 이동(MoveJoint)")

    elif t == "task":
        pos = [float(p.get(k, 0.0)) for k in ["x","y","z","rx","ry","rz"]]
        vel, acc = float(p.get("vel", 100.0)), float(p.get("acc", 200.0))
        req = MoveLine.Request()
        req.pos = pos; req.vel = [vel, vel]; req.acc = [acc, acc]
        req.time = 0.0; req.radius = 0.0; req.ref = 0; req.mode = 0; req.blend_type = 0; req.sync_type = 0
        log.info(f"태스크 이동: {pos} vel={vel} acc={acc}")
        call_service(node, MoveLine, SRV_MOVE_TASK, req, "태스크 이동(MoveLine)")

    elif t == "gripper":
        action = p.get("action", "open")
        try:
            from DSR_ROBOT2 import set_digital_output
            if action == "close":
                set_digital_output(1, 1)  # Grasp ON
                set_digital_output(2, 0)  # Release OFF
            else:
                set_digital_output(2, 1)  # Release ON
                set_digital_output(1, 0)  # Grasp OFF
            log.info(f"✅ 그리퍼 {action} 완료")
        except Exception as e:
            log.error(f"그리퍼 제어 실패: {e}")

    elif t == "recover":
        # 비상정지/서보꺼짐(state 3,6) → 서보 ON 복구
        log.info("🔄 복구 시도 (서보 ON)")
        rt_set(DB_STATE, "recovering")
        def _do_recover():
            time.sleep(0.5)
            ok = call_robot_control(node, 3, "서보ON(recover)")
            if ok:
                time.sleep(3.0)  # 브레이크 해제 대기 (철컥 소리)
                log.info("✅ 복구 완료")
                rt_set(DB_STATE, "idle")
            else:
                log.error("❌ 복구 실패 — 물리적 조치 필요")
                rt_set(DB_STATE, "estop")
        threading.Thread(target=_do_recover, daemon=True).start()

    elif t == "safe_stop_reset":
        # 보호정지(state 5) → 해제
        log.info("🔄 보호정지 해제 시도")
        rt_set(DB_STATE, "recovering")
        def _do_safe_reset():
            time.sleep(0.5)
            ok = call_robot_control(node, 2, "보호정지해제")
            if ok:
                time.sleep(2.0)
                log.info("✅ 보호정지 해제 완료")
                rt_set(DB_STATE, "idle")
            else:
                log.error("❌ 보호정지 해제 실패")
                rt_set(DB_STATE, "safe_stop")
        threading.Thread(target=_do_safe_reset, daemon=True).start()

    elif t == "recovery_mode_on":
        # ── 복구 모드 진입 ────────────────────────────────────────────
        # SetSafetyMode(RECOVERY, ENTER) 호출 → 보호정지 상태에서도
        # vel≤30 조건으로 MoveJoint/MoveLine 명령이 통함.
        # 티치펜던트의 "복구 모드" 버튼과 동일한 동작.
        # ─────────────────────────────────────────────────────────────
        log.info("🟡 복구 모드 진입 시도")
        rt_set(DB_STATE, "recovery_mode")
        rt_set(DB_RECOVERY, {"active": True, "msg": "복구 모드 진입 중..."})

        def _enter_recovery():
            try:
                from dsr_msgs2.srv import SetSafetyMode
            except ImportError:
                log.error("SetSafetyMode srv 없음 — dsr_msgs2 버전 확인 필요")
                rt_set(DB_RECOVERY, {"active": False, "msg": "SetSafetyMode 서비스 없음"})
                rt_set(DB_STATE, "safe_stop")
                return

            cli = node.create_client(SetSafetyMode, SRV_SET_SAFETY_MODE)
            if not cli.wait_for_service(timeout_sec=1.0):
                log.error("❌ SetSafetyMode 서비스 응답 없음")
                rt_set(DB_RECOVERY, {"active": False, "msg": "컨트롤러 응답 없음"})
                rt_set(DB_STATE, "safe_stop")
                return

            req = SetSafetyMode.Request()
            req.safety_mode  = SAFETY_MODE_RECOVERY
            req.safety_event = SAFETY_MODE_EVENT_ENTER
            future = cli.call_async(req)

            deadline = time.time() + 5.0
            while not future.done():
                if time.time() > deadline:
                    log.error("❌ SetSafetyMode(RECOVERY) 타임아웃")
                    rt_set(DB_RECOVERY, {"active": False, "msg": "타임아웃 — 다시 시도하세요"})
                    rt_set(DB_STATE, "safe_stop")
                    return
                time.sleep(0.05)

            res = future.result()
            if res and res.success:
                log.info("✅ 복구 모드 진입 완료 — 서보 ON 시도")
                # 티치펜던트와 동일한 순서: 복구 모드 진입 → 서보 ON → jog 가능
                time.sleep(0.5)
                servo_ok = call_robot_control(node, 3, "서보ON(복구모드)")
                if servo_ok:
                    log.info("✅ 서보 ON 완료 — 조그 가능")
                    # 서보 ON 후 movej_h2r / movel_h2r 액션 서버가 생성됨 → 클라이언트 연결
                    try:
                        node._movej_h2r_cli = ActionClient(node, MovejH2r, '/dsr01/motion/movej_h2r')
                        node._movel_h2r_cli = ActionClient(node, MovelH2r, '/dsr01/motion/movel_h2r')
                        log.info("MovejH2r/MovelH2r 액션 클라이언트 등록")
                    except Exception as e:
                        log.error(f"H2r 액션 클라이언트 등록 실패: {e}")
                    rt_set(DB_RECOVERY, {
                        "active": True,
                        "msg": "복구 모드 활성\n조그로 외력 제거 후 '복구 완료'를 누르세요"
                    })
                else:
                    log.error("❌ 서보 ON 실패")
                    rt_set(DB_RECOVERY, {"active": False, "msg": "서보 ON 실패 — 다시 시도하세요"})
                    rt_set(DB_STATE, "safe_stop")
            else:
                log.error("❌ 복구 모드 진입 실패")
                rt_set(DB_RECOVERY, {"active": False, "msg": "복구 모드 진입 실패 — 컨트롤러 상태 확인"})
                rt_set(DB_STATE, "safe_stop")

        threading.Thread(target=_enter_recovery, daemon=True).start()

    elif t == "recovery_mode_off":
        # ── 복구 모드 해제 → 자율 모드 복귀 + 보호정지 해제 ──────────
        log.info("🔵 복구 모드 해제 → 자율 모드 복귀")
        rt_set(DB_RECOVERY, {"active": False, "msg": "자율 모드 복귀 중..."})

        def _exit_recovery():
            # ① 진행 중인 H2r 액션 먼저 취소
            if node._recovery_goal_handle is not None:
                try:
                    node._recovery_goal_handle.cancel_goal_async()
                    log.info("복구 모드 해제: 진행 중인 이동 취소")
                except Exception as e:
                    log.error(f"이동 취소 실패: {e}")
                node._recovery_goal_handle = None
            time.sleep(0.5)  # 취소 완료 대기

            try:
                from dsr_msgs2.srv import SetSafetyMode
            except ImportError:
                rt_set(DB_RECOVERY, {"active": False, "msg": "SetSafetyMode 서비스 없음"})
                return

            # ② SetSafetyMode(AUTONOMOUS) — 정상 모드 복귀
            cli = node.create_client(SetSafetyMode, SRV_SET_SAFETY_MODE)
            if not cli.wait_for_service(timeout_sec=1.0):
                rt_set(DB_RECOVERY, {"active": False, "msg": "컨트롤러 응답 없음"})
                return

            req = SetSafetyMode.Request()
            req.safety_mode  = SAFETY_MODE_AUTONOMOUS
            req.safety_event = SAFETY_MODE_EVENT_ENTER
            future = cli.call_async(req)

            deadline = time.time() + 5.0
            while not future.done():
                if time.time() > deadline:
                    rt_set(DB_RECOVERY, {"active": False, "msg": "자율 모드 전환 타임아웃"})
                    return
                time.sleep(0.05)

            log.info("✅ AUTONOMOUS 모드 복귀")
            time.sleep(0.5)

            # ② SetRobotControl(2) — 보호정지 해제
            ok = call_robot_control(node, 2, "보호정지해제(recovery_off)")
            if ok:
                time.sleep(2.0)
                # 재발 여부 1회 체크
                try:
                    if DSR_ROBOT2_AVAILABLE:
                        from DSR_ROBOT2 import get_robot_state
                        code = int(get_robot_state())
                    else:
                        code = int(rt_get(DB_ROBOT_STATE) or -1)
                except Exception:
                    code = -1

                if code == 5:
                    log.warning("⚠ 보호정지 재발 — 외력이 남아 있음")
                    rt_set(DB_STATE, "safe_stop")
                    rt_set(DB_RECOVERY, {"active": False, "msg": "보호정지 재발\n외력이 남아 있습니다\n복구 모드를 다시 시작하세요"})
                else:
                    log.info("✅ 복구 완료 → idle")
                    rt_set(DB_STATE, "idle")
                    rt_set(DB_RECOVERY, {"active": False, "msg": "복구 완료"})
            else:
                log.warning("보호정지 해제 실패 — 이미 해제 상태일 수 있음")
                rt_set(DB_STATE, "idle")
                rt_set(DB_RECOVERY, {"active": False, "msg": "복구 완료"})

        threading.Thread(target=_exit_recovery, daemon=True).start()

# ══════════════════════════════════════════════════════
# 명령 폴링
# ══════════════════════════════════════════════════════
_last_cmd_ts = _last_ctrl_ts = int(time.time() * 1000)  # 시작 전 명령 무시

def poll_commands(node):
    global _last_cmd_ts
    data = rt_get(DB_COMMAND)
    if not data or not isinstance(data, dict): return
    ts = data.get("sentAt", 0)
    if ts <= _last_cmd_ts: return
    _last_cmd_ts = ts
    cmd = data.get("cmd", "")
    log.info(f"검사 명령: {cmd}")
    if cmd == "00":
        log.info("▶ [검사시작] /dsr01/progress_status 로 '00' publish")
        msg = String()
        msg.data = "00"
        node._progress_pub.publish(msg)
        return
    if cmd == "stop":
        log.info("⏹ [검사중지] 일시정지 → 홈복귀 → 초기화")

        # 1. 일시정지 (컨트롤러 레벨 — 다른 노드 모션도 즉시 멈춤)
        call_service(node, MovePause, SRV_PAUSE, MovePause.Request(), "검사중지(MovePause)")

        time.sleep(0.5)  # pause 안착 대기

        try:
            from DSR_ROBOT2 import drl_script_stop, DR_QSTOP_STO
            drl_script_stop(DR_QSTOP_STO)
            log.info("✅ DRL 스크립트 종료")
        except Exception as e:
            log.warning(f"drl_script_stop 실패, DrlStop 서비스로 폴백: {e}")
            call_service(node, DrlStop, SRV_DRL_STOP, DrlStop.Request(), "DrlStop 폴백")

        time.sleep(0.5)  # 스크립트 종료 안착 대기

        # 3. 홈 복귀
        log.info("🏠 홈 복귀")
        home = {"j1": 0.0, "j2": 0.0, "j3": 90.0, "j4": 0.0, "j5": 90.0, "j6": 0.0, "vel": 40.0, "acc": 40.0}
        execute_control(node, {"type": "joint", "payload": home})

        # 4. 상태 초기화 (재검사 가능)
        rt_set(DB_STATE, "idle")
        log.info("✅ 검사중지 완료 — 재검사 가능 상태")

def poll_robot_state(node):
    """로봇 상태코드 + 태스크 좌표 → Firebase
    /dsr01/state 토픽이 발행 안 되므로 get_current_posx() 폴링으로 대체"""
    if not DSR_ROBOT2_AVAILABLE: return
    try:
        from DSR_ROBOT2 import get_robot_state, get_current_posx
        # ── 상태 코드 ──
        code = get_robot_state()
        rt_set(DB_ROBOT_STATE, code)
        if code == 5:
            rt_set(DB_STATE, "safe_stop")
        elif code in (3, 6, 9, 10):
            current = rt_get(DB_STATE)
            if current not in ("estop", "recovering"):
                rt_set(DB_STATE, "estop")
        # ── 태스크 좌표 (/dsr01/state 토픽 미발행 → 폴링으로 대체) ──
        try:
            posx_raw = get_current_posx()[0]
            px = [round(float(v), 2) for v in posx_raw]
            rt_set(DB_TASK, {
                "x": px[0], "y": px[1], "z": px[2],
                "rx": px[3], "ry": px[4], "rz": px[5],
                "updatedAt": int(datetime.now().timestamp() * 1000),
            })
        except Exception as e:
            log.error(f"태스크 좌표 폴링 오류: {e}")
    except Exception as e:
        log.error(f"로봇 상태 폴링 오류: {e}")


def poll_control(node):
    global _last_ctrl_ts
    data = rt_get(DB_CONTROL)
    if not data or not isinstance(data, dict): return
    ts = data.get("sentAt", 0)
    if ts <= _last_ctrl_ts: return
    _last_ctrl_ts = ts
    execute_control(node, data)

# ══════════════════════════════════════════════════════
# ROS2 노드
# ══════════════════════════════════════════════════════
class InspectionBridgeNode(Node):
    def __init__(self):
        super().__init__("inspection_firebase_bridge")

        self.create_subscription(String, TOPIC_ROBOT_STATUS,   self._cb_status,  10)
        self.create_subscription(String, TOPIC_INSP_RESULT,    self._cb_result,  10)

        # 검사 명령 publisher (웹 → /dsr01/progress_status)
        self._progress_pub = self.create_publisher(String, TOPIC_ROBOT_STATUS, 10)


        # 조인트 상태 구독 → robot/joints
        try:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, "/dsr01/joint_states", self._cb_joint_states, 10)
            log.info("JointState 구독 등록")
        except ImportError:
            log.warning("sensor_msgs 없음")

        # /dsr01/state 토픽 미발행 확인 → 태스크 좌표는 poll_robot_state에서 폴링으로 처리

        # Jog 서비스 클라이언트 (일반 모드 조그)
        if DSR_MSGS_AVAILABLE:
            try:
                self._jog_cli = self.create_client(Jog, '/dsr01/motion/jog')
                log.info("Jog 서비스 클라이언트 등록")
            except Exception as e:
                self._jog_cli = None
                log.warning(f"Jog 서비스 클라이언트 등록 실패: {e}")
        else:
            self._jog_cli = None

        # MovejH2r/MovelH2r 액션 클라이언트 (복구 모드 전용 — 서보ON 후 생성됨)
        # 복구 모드 진입 시 동적으로 생성하므로 초기값은 None
        self._movej_h2r_cli = None
        self._movel_h2r_cli = None
        self._recovery_goal_handle = None  # 현재 활성 복구 이동 핸들

        self.create_timer(0.2, lambda: poll_control(self))
        self.create_timer(1.0, lambda: poll_commands(self))
        self.create_timer(1.0, lambda: poll_robot_state(self))  # 로봇 상태 모니터링

        # ★ 브리지 시작 시 로봇 상태 1회 즉시 동기화
        # DSR_ROBOT2는 init_dsr_robot2()에서 별도 스레드로 로드되므로
        # 로드 완료(약 1.5초) 후 실행되도록 2초 딜레이 타이머 사용
        self._init_sync_done = False
        self.create_timer(2.0, self._initial_state_sync)

        self.get_logger().info("Firebase 브리지 노드 초기화 완료")

    def _initial_state_sync(self):
        """브리지 최초 기동 시 로봇 현재값을 Firebase에 1회 강제 동기화.
        - robot/robot_state : 로봇 상태 코드
        - robot/state       : safe_stop / estop / idle
        - robot/joints      : 현재 조인트 각도 (get_current_posj)
        - robot/task        : 현재 태스크 좌표 (get_current_posx)
        """
        if self._init_sync_done:
            return
        self._init_sync_done = True  # 1회만 실행

        if not DSR_ROBOT2_AVAILABLE:
            log.warning("초기 상태 동기화: DSR_ROBOT2 미사용 — 건너뜀")
            return

        try:
            from DSR_ROBOT2 import get_robot_state, get_current_posj, get_current_posx

            # ── 1. 로봇 상태 코드 ──────────────────────────────
            code = int(get_robot_state())
            log.info(f"[초기 동기화] 로봇 상태 코드: {code}")
            rt_set(DB_ROBOT_STATE, code)

            # 5 → 보호정지, 3/6 → 비상정지/서보꺼짐, 그 외 → idle
            if code == 5:
                rt_set(DB_STATE, "safe_stop")
                log.info("[초기 동기화] → safe_stop 반영")
            elif code in (3, 6, 9, 10):
                rt_set(DB_STATE, "estop")
                log.info("[초기 동기화] → estop 반영")
            else:
                rt_set(DB_STATE, "idle")
                log.info(f"[초기 동기화] 정상 상태({code}) → idle 반영")

            # ── 2. 조인트 각도 ─────────────────────────────────
            try:
                posj = get_current_posj()   # [j1, j2, j3, j4, j5, j6] (deg)
                joints_data = {
                    "j1": round(posj[0], 2), "j2": round(posj[1], 2),
                    "j3": round(posj[2], 2), "j4": round(posj[3], 2),
                    "j5": round(posj[4], 2), "j6": round(posj[5], 2),
                    "updatedAt": int(datetime.now().timestamp() * 1000),
                }
                rt_set(DB_JOINTS, joints_data)
                log.info(f"[초기 동기화] 조인트: {[round(v,1) for v in posj]}")
            except Exception as e:
                log.error(f"[초기 동기화] 조인트 읽기 실패: {e}")

            # ── 3. 태스크(직교) 좌표 ───────────────────────────
            try:
                # get_current_posx() → (posx객체[x,y,z,rx,ry,rz], 참조좌표계) 튜플 반환
                posx_raw = get_current_posx()[0]
                px = [round(float(v), 2) for v in posx_raw]
                task_data = {
                    "x":  px[0], "y":  px[1], "z":  px[2],
                    "rx": px[3], "ry": px[4], "rz": px[5],
                    "updatedAt": int(datetime.now().timestamp() * 1000),
                }
                rt_set(DB_TASK, task_data)
                log.info(f"[초기 동기화] 태스크: {px}")
            except Exception as e:
                log.error(f"[초기 동기화] 태스크 읽기 실패: {e}")

        except Exception as e:
            log.error(f"[초기 동기화] 실패: {e}")

    # ── 검사 콜백 ─────────────────────────────────────
    def _cb_status(self, msg):
        state_map = {
            "start_move":"moving", "start_insp":"inspecting",
            "return":"moving",     "idle":"idle",
            "01":"moving",         # 아두이노: 물건 이동 중
            "02":"inspecting",     # 로봇: 검사 시작
            "04":"idle",           # 로봇: 이동 완료 후 대기
        }
        rt_set(DB_STATE, state_map.get(msg.data, msg.data))

    def _cb_result(self, msg):
        session.total += 1
        if msg.data == "PASS":
            session.passed += 1
            rt_set(DB_STATE, "pass")
            push_stats()          # 합격: 통계만 업데이트
        elif msg.data in ("FAIL", "defect"):
            session.failed += 1
            rt_set(DB_STATE, "defect")  # 웹 상태 UI → 불량 감지
            push_stats()          # 통계 업데이트
            # robot/defect 에 storageUrl 쓰기는 팀원 코드가 직접 수행



    # ── 조인트 상태 → robot/joints ────────────────────
    def _cb_joint_states(self, msg):
        now = time.time()
        if not hasattr(self, '_last_joint_write'):
            self._last_joint_write = 0.0

        # ★ 0.033초(약 30Hz) 미만이면 스킵
        if now - self._last_joint_write < 0.033:
            return
        self._last_joint_write = now
        try:
            # name 순서가 [joint_1,joint_2,joint_4,joint_5,joint_3,joint_6] 등
            # 뒤섞여 있으므로 name 기준으로 매핑
            name_to_key = {
                "joint_1":"j1","joint_2":"j2","joint_3":"j3",
                "joint_4":"j4","joint_5":"j5","joint_6":"j6"
            }
            data = {}
            for i, name in enumerate(msg.name[:6]):
                key = name_to_key.get(name)
                if key is None: continue
                val = msg.position[i]
                # 라디안 여부 판단: -7~7 범위면 라디안
                if abs(val) <= 7.0:
                    val = math.degrees(val)
                data[key] = round(val, 2)
            data["updatedAt"] = int(datetime.now().timestamp() * 1000)
            rt_set(DB_JOINTS, data)
        except Exception as e:
            self.get_logger().error(f"조인트 상태 오류: {e}")

    # ── 로봇 상태(TCP 위치 + 상태코드) → robot/task, robot/robot_state ──
    def _cb_robot_state(self, msg):
        """
        dsr_msgs/msg/RobotState.current_posx: float64[6]
        순서: [X(mm), Y(mm), Z(mm), RX(deg), RY(deg), RZ(deg)]
        """
        try:
            p = msg.current_posx  # float64[6]
            rt_set(DB_TASK, {
                "x":  round(p[0], 2),
                "y":  round(p[1], 2),
                "z":  round(p[2], 2),
                "rx": round(p[3], 2),
                "ry": round(p[4], 2),
                "rz": round(p[5], 2),
                "updatedAt": int(datetime.now().timestamp() * 1000),
            })
            # 로봇 상태 코드 → robot/robot_state (웹 복구 버튼 활성화용)
            if hasattr(msg, 'robot_state'):
                code = int(msg.robot_state)
                rt_set(DB_ROBOT_STATE, code)
                # 비정상 상태 자동 감지
                if code == 5:
                    rt_set(DB_STATE, "safe_stop")
                elif code in (3, 6, 9, 10):
                    current = rt_get(DB_STATE)
                    if current not in ("estop", "recovering"):
                        rt_set(DB_STATE, "estop")
        except Exception as e:
            self.get_logger().error(f"태스크 좌표 오류: {e}")

# ══════════════════════════════════════════════════════
# 진입점
# ══════════════════════════════════════════════════════
if __name__ == "__main__":
    log.info("=" * 55)
    log.info("DSR M0609 Firebase 브리지 시작")
    log.info(f"Firebase  : {'사용 가능' if FIREBASE_AVAILABLE else '❌ 설치 필요'}")
    log.info(f"ROS2      : {'사용 가능' if ROS_AVAILABLE else '❌ ROS2 환경 필요'}")
    log.info(f"dsr_msgs  : {'사용 가능' if DSR_MSGS_AVAILABLE else '⚠ 없음 (토픽 폴백)'}") 
    log.info(f"DSR_ROBOT2: {'사용 가능' if DSR_ROBOT2_AVAILABLE else '⚠ 없음'}")
    log.info("=" * 55)

    if not FIREBASE_AVAILABLE or not ROS_AVAILABLE:
        log.error("필수 패키지가 없습니다. 종료합니다."); exit(1)
    if not init_firebase():
        log.error("Firebase 초기화 실패."); exit(1)

    rclpy.init()
    node = InspectionBridgeNode()
    init_dsr_robot2()  # DSR_ROBOT2 별도 스레드로 초기화
    try:
        log.info("브리지 실행 중... (Ctrl+C로 종료)")
        rclpy.spin(node)
    except KeyboardInterrupt:
        log.info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()
