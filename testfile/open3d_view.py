import open3d as o3d
import numpy as np

def main():
    print("🚀 Open3D 뷰어 시작 (실제 데이터 기반 Mesh 생성)...")

    # 1. 로컬에 저장된 npy 파일 불러오기
    try:
        pts = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
        print(f"✅ baseline.npy 로드 완료 (총 {len(pts)}개의 점)")
    except FileNotFoundError:
        print("❌ baseline.npy 파일이 없습니다. 먼저 기록 코드를 실행해주세요.")
        return

    # ==========================================================
    # 2. 로봇 궤적을 선(LineSet)으로 만들기 (초록색)
    # ==========================================================
    lines = [[i, i + 1] for i in range(len(pts) - 1)]
    colors = [[0.0, 0.8, 0.0] for _ in range(len(lines))] # 진한 초록색
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # ==========================================================
    # 3. 💡 [핵심] 실제 점들을 감싸는 진짜 면(Mesh) 생성
    # Convex Hull 방식을 사용하여 점들의 바깥쪽을 비닐 랩 씌우듯 감싸줍니다.
    # ==========================================================
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    
    hull_mesh, _ = pcd.compute_convex_hull()
    hull_mesh.compute_vertex_normals() # 빛 반사를 위해 입체감 부여
    hull_mesh.paint_uniform_color([0.4, 0.6, 0.9]) # 옅은 파란색 칠하기

    # ==========================================================
    # 4. 좌표계 축 표시 (로봇이 처음 시작한 위치 근처에 표시)
    # ==========================================================
    origin_pt = pts[0] # 첫 번째 궤적 점을 기준으로 축 생성
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50.0, origin=origin_pt)

    # ==========================================================
    # 5. Open3D 창 띄우기
    # ==========================================================
    print("👀 마우스로 화면을 돌려보세요!")
    print("💡 꿀팁: 창이 켜진 상태에서 키보드 'W'를 누르면 면이 투명한 그물망(Wireframe)으로 바뀝니다!")
    
    o3d.visualization.draw_geometries(
        [line_set, hull_mesh, axes],
        window_name="Real Scanned Mesh Viewer",
        width=1024, height=768,
        mesh_show_back_face=True # 면의 안팎 모두 렌더링
    )

if __name__ == "__main__":
    main()