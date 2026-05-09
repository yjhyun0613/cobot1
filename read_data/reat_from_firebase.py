import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# 1. Firebase Admin SDK 초기화
# 1단계 5번에서 다운로드한 json 키 파일 경로
SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
# 1단계 4번의 databaseURL 값 (Firebase 콘솔 -> Realtime Database에서 확인)
DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app  " 


try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DATABASE_URL
    })
except ValueError:
    print("Firebase 앱이 이미 초기화되었습니다.")

# 2. 리스너 콜백 함수 정의
# DB에서 데이터가 변경될 때마다 *자동으로* 이 함수가 호출됩니다.
def listener_callback(event):
    """
    event.event_type: 'put' (데이터 생성/덮어쓰기), 'patch' (데이터 업데이트) 등
    event.path: 변경된 데이터의 경로 (예: '/completed_jobs')
    event.data: 변경된 후의 데이터
    """
    print("----- 데이터 변경 감지! -----")
    print(f"이벤트 타입: {event.event_type}")
    print(f"경로: {event.path}")
    print(f"새 데이터: {event.data}")
    print("---------------------------------")

# 3. 데이터베이스 리스너 설정
# '/robot_status' 경로의 모든 변경 사항을 감지합니다.
ref = db.reference('/robot_status')
ref.listen(listener_callback)

print("Firebase DB 리스너 시작. 데이터 변경을 대기합니다... (Ctrl+C로 종료)")

# 4. 스크립트가 종료되지 않도록 유지
# 리스너는 백그라운드 스레드에서 실행되므로, 메인 스레드가 종료되는 것을 방지합니다.
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")