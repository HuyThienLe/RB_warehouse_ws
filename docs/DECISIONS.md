# QUYẾT ĐỊNH THIẾT KẾ (DECISIONS LOG)

## Bước 1: Môi trường và Khung Workspace
1. **Quản lý phiên bản thư viện Python:**
   - Ghim `numpy<2` (thực tế cài `1.26.4`) để tránh xung đột ABI với `cv_bridge` (vốn biên dịch với NumPy 1.x C-API).
   - Tuyệt đối không cài các gói PyPI `opencv-python` hay `opencv-contrib-python` vào môi trường toàn cục/user để không đè thư viện `cv2` hệ thống từ `python3-opencv` (v4.9.0 có sẵn module ArUco).
2. **Cấu trúc phân rã tham số gốc:**
   - Tạo `wh_description/config/robot_params.yaml` cho toàn bộ thông số hình học, động học, giới hạn khớp và cảm biến robot.
   - Tạo `wh_gazebo/config/world_params.yaml` cho toàn bộ thông số kích thước sa bàn kho, vị trí kệ hàng, trạm và tag ArUco.
   - Toàn bộ các bước sau phải tham chiếu từ 2 file YAML này, không hardcode giá trị trong mã nguồn.
3. **Cấu trúc Install Directory:**
   - Chỉ chỉ định các thư mục thực sự tồn tại vào `CMakeLists.txt` trong lệnh `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})`, kết hợp file `.gitkeep` để đảm bảo git và colcon build không bị lỗi đường dẫn rỗng.
