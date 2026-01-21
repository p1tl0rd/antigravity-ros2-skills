# ANTIGRAVITY AGENT STRESS TEST MATRIX

Mục tiêu: Đảm bảo bộ cấu hình `.agent` (Rules, Skills, Workflows) hoạt động đúng trong các tình huống phức tạp thực tế.

## ✅ Test Case 1: The "Simple" Python Node (Completed)
- **Status**: PASSED
- **Verified**: `ros2-python-style`, `setup.cfg` generation.
- **Fixes Applied**: Prevent `setup.cfg` overwrite.

## ⬜ Test Case 2: The "Pure" Interface Package
- **Goal**: Kiểm tra quy tắc `ros2-architecture` về việc tách biệt gói tin interface.
- **Input**: Tạo gói `antigravity_interfaces` chứa 1 Msg và 1 Srv.
- **Fail Condition**: Gói tin chứa cả mã nguồn executable hoặc thiếu dependency `rosidl`.

## ⬜ Test Case 3: The "Complex" C++ Lifecycle Component
- **Goal**: Kiểm tra kỹ năng `ros2_package_scaffolder` (C++) và `ros2-cpp-style`.
- **Input**: Tạo gói `antigravity_cpp_node`.
- **Requirements**:
    - Phải là Shared Library (không có `main` cứng).
    - Phải dùng `LifecycleNode`.
    - Phải pass `ament_lint_auto` (cpplint, uncrustify).
    - Phải link được với `antigravity_interfaces` (Test Case 2).

## ⬜ Test Case 4: Workflow Integration (CI/CD)
- **Goal**: Chạy toàn bộ quy trình `/ci-local-pipeline`.
- **Expectation**: Tất cả các gói (Python test, Interface, C++ test) đều build và pass test.

## ⬜ Test Case 5: Rule Violation Check (Negative Testing)
- **Goal**: Cố tình tạo một file vi phạm quy tắc (ví dụ: đặt tên file C++ sai, dùng raw pointer).
- **Expectation**: Workflow CI phải phát hiện và báo lỗi.
