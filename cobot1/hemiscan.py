# 크랙 검사 지표 계산을 위한 Python 코드입니다.
import numpy as np
from scipy.interpolate import CubicSpline

def extract_and_generate_paths(raw_time, raw_positions, raw_torques, window_size=15, margin=20):
    """
    전체 로봇 주행 데이터에서 크랙 구간(actual_path)을 추출하고,
    정상 구간 데이터를 바탕으로 이상적인 경로(ideal_path)를 생성하는 함수
    
    :param raw_time: 전체 시간 데이터 배열 (N,)
    :param raw_positions: 전체 3D 좌표 데이터 배열 (N, 3)
    :param raw_torques: 전체 토크 데이터 배열 (N,)
    :param window_size: 특이점을 기준으로 앞뒤 몇 개의 데이터를 에러 구간으로 잡을지
    :param margin: 이상적 곡선을 그리기 위해 가져올 에러 구간 앞뒤의 정상 데이터 개수
    """
    
    # 1. 특이점(크랙) 찾기: 토크값이 가장 크게 튄 지점의 인덱스 추출
    spike_idx = np.argmax(np.abs(raw_torques))
    
    # (주의: 실제 환경에서는 spike_idx가 배열 양끝에 너무 가까우면 에러가 날 수 있으므로 예외 처리가 필요합니다)
    
    # 2. 에러 구간(Window) 설정: 크랙의 영향을 받은 실제 궤적 (actual_path)
    err_start = spike_idx - window_size
    err_end = spike_idx + window_size
    
    t_actual = raw_time[err_start:err_end]
    actual_path = raw_positions[err_start:err_end]
    actual_torques = raw_torques[err_start:err_end]
    
    # 3. 정상 구간 데이터 추출: 이상적 궤적을 추정하기 위한 '기준점'들
    # 에러 구간을 제외한 바로 앞(margin)과 바로 뒤(margin)의 데이터를 합칩니다.
    normal_idx = list(range(err_start - margin, err_start)) + \
                 list(range(err_end, err_end + margin))
                 
    t_normal = raw_time[normal_idx]
    p_normal = raw_positions[normal_idx] # (N, 3) 형태의 정상 구간 3D 좌표
    
    # 4. 곡선 근사화 (Curve Fitting) 및 이상적인 궤적(ideal_path) 생성
    # 시간(t_normal)에 따른 각 X, Y, Z 좌표(p_normal)의 변화를 3차 곡선으로 학습
    spline_model = CubicSpline(t_normal, p_normal)
    
    # 학습된 곡선 모델에 에러 구간의 시간(t_actual)을 입력하여 '원래 있어야 할' 좌표 예측
    ideal_path = spline_model(t_actual)
    
    return actual_path, ideal_path, actual_torques

# =====================================================================
# 🛠 테스트를 위한 가상 데이터 생성 (실제로는 로봇에서 받아온 데이터를 넣으시면 됩니다)
# =====================================================================
np.random.seed(42)
total_samples = 500
raw_time = np.linspace(0, 5, total_samples) # 5초 동안 500개 샘플 (100Hz)

# 반구형 아크릴을 긁는 부드러운 곡선 궤적 (가상 X, Y, Z)
x_base = np.linspace(0, 100, total_samples)
y_base = 50 * np.sin(raw_time)
z_base = -10 * (raw_time - 2.5)**2 + 100  # 위로 볼록한 반구 형태

raw_positions = np.column_stack((x_base, y_base, z_base))
raw_torques = np.random.normal(0, 0.1, total_samples) # 평상시 잔잔한 토크 (노이즈)

# 특정 지점(크랙)에서 좌표가 미세하게 튀고, 토크가 확 튀는 상황 연출
crack_index = 250
raw_positions[crack_index-5:crack_index+5, 2] += np.random.uniform(-1.5, 1.5, 10) # Z축 덜덜거림
raw_torques[crack_index-2:crack_index+3] += [2.0, 8.5, 15.0, -10.2, 3.1] # 토크 스파이크!

# =====================================================================
# 🚀 실행 및 이전 답변의 '오차 계산 함수'와 연동
# =====================================================================

# 1. 궤적 추출 및 추정
actual_path, ideal_path, window_torques = extract_and_generate_paths(
    raw_time, raw_positions, raw_torques, window_size=15, margin=20
)

# 2. (이전 답변의 함수) 오차 지표 계산
dt = raw_time[1] - raw_time[0] # 샘플링 주기 계산
distances = np.linalg.norm(actual_path - ideal_path, axis=1)
e_max = np.max(distances)
rmse = np.sqrt(np.mean(distances**2))

print("=== 궤적 추출 및 분석 결과 ===")
print(f"추출된 데이터 샘플 수: {len(actual_path)}개")
print(f"최대 위치 편차 (Max Deviation): {e_max:.4f} mm")
print(f"평균 제곱근 오차 (RMSE): {rmse:.4f} mm")