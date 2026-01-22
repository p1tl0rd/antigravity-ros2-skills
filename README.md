# Antigravity ROS 2 Skills

🚀 Bộ cấu hình "Agent-First" biến IDE của bạn thành môi trường phát triển Robot chuyên nghiệp với AI Agents (Google Gemini, Claude, Cursor, etc.).

---

## 📦 Cài Đặt

### Bước 1: Clone vào ROS 2 Workspace
```bash
# Tạo workspace mới (hoặc dùng workspace có sẵn)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone repository vào ROOT của workspace (không phải /src)
git clone https://github.com/p1tl0rd/antigravity-ros2-skills.git .
```

### Bước 2: Cài đặt Python Dependencies
```bash
pip install -r requirements.txt
```

### Bước 3: Khởi tạo Linter Configs
```bash
python3 src/tools/setup_linter.py
```
> Script này sẽ tạo `setup.cfg` (Flake8) và `.clang-format` (C++) ở thư mục gốc.

### Bước 4: Cài đặt ROS 2 Dependencies
```bash
# Khởi tạo rosdep (nếu chưa có)
sudo rosdep init  # Bỏ qua nếu đã chạy trước đó
rosdep update

# Cài dependencies cho các package trong src/
rosdep install --from-paths src --ignore-src -r -y
```

### Bước 5: Restart IDE
Khởi động lại IDE (Cursor/VSCode) để nhận diện thư mục `.agent` và MCP Servers.

---

## 🚀 Hướng Dẫn Sử Dụng

**Nguyên tắc cốt lõi**: Bạn cứ giao tiếp tự nhiên, Agent sẽ tự động chọn công cụ phù hợp.

### 1. Giao Tiếp Tự Nhiên (Khuyên dùng)
Bạn không cần nhớ tên Skill hay Rule. Chỉ cần mô tả mục tiêu:

*   *"Tạo cho tôi một package Python để điều khiển động cơ qua cổng Serial."*
    → Agent tự kích hoạt `ros2_package_scaffolder`.
*   *"Code này nhìn rối quá, review giúp tôi."*
    → Agent tự dùng `code-review`.
*   *"Build lỗi rồi, fix đi."*
    → Agent tự dùng `ros2-build-resolver`.

### 2. Quy Tắc Tự Động (Automation)
Bạn không cần nhắc Agent tuân thủ chuẩn. Chỉ cần viết code, Agent sẽ tự động:
*   ✅ Áp dụng OOP cho Node (theo `ros2-architecture`).
*   ✅ Format code chuẩn PEP 8 / Google Style.
*   ✅ Thêm Docstring và Type Hinting.

### 3. Power Users (Slash Commands)
Nếu muốn đi tắt đón đầu, bạn có thể dùng lệnh:
*   `/ci-local-pipeline`: Chạy full bộ test.
*   `/clean-init`: Xóa sạch workspace làm lại từ đầu.

---

## 📋 Cấu Trúc Repository

```
antigravity-ros2-skills/
├── .agent/                    # 🧠 Agent Configuration
│   ├── rules/                 # Quy tắc code style (luôn bật)
│   ├── skills/                # Kỹ năng chuyên biệt (gọi khi cần)
│   └── workflows/             # Quy trình step-by-step (slash commands)
├── .github/workflows/         # 🔄 GitHub Actions CI/CD
├── src/tools/                 # 🛠️ Scripts hỗ trợ
├── templates/                 # 📄 Mẫu tham khảo
├── mcp_servers.json           # MCP Server config
├── requirements.txt           # Python dependencies
└── README.md
```

---

## � Chi Tiết Các Thành Phần

### 📐 Rules (Quy tắc - Tự động áp dụng)
Rules được tự động kích hoạt dựa trên file đang mở. Agent sẽ tuân thủ các quy tắc này khi viết code.

| Rule | Trigger | Mô tả |
|:-----|:--------|:------|
| **ros2-architecture** | Luôn bật | Phân tách Build Type (C++/Python/Interface). Yêu cầu Component-based Design. |
| **ros2-cpp-style** | `*.cpp`, `*.hpp`, `*.h` | OOP bắt buộc. Naming conventions (`snake_case` method, `member_` variables). Không dùng `new/delete`. |
| **ros2-python-style** | `*.py` | PEP 8. Node phải kế thừa `rclpy.node.Node`. Entry point chuẩn. |
| **technical-standards** | Luôn bật | KISS, DRY, YAGNI. Type Safety. Error Handling. Async best practices. |

### 🛠️ Skills (Kỹ năng - Gọi khi cần)
Skills là "chuyên gia" mà bạn có thể triệu hồi bằng cách yêu cầu Agent sử dụng.

| Skill | Mô tả | Khi nào dùng |
|:------|:------|:-------------|
| **ros2_package_scaffolder** | Tạo package ROS 2 chuẩn (C++/Python/Interface) | Khi bắt đầu package mới |
| **ros2_build_master** | Quản lý build với Colcon (options tối ưu) | Khi cần build nâng cao |
| **lifecycle_operator** | Điều khiển Lifecycle Nodes (configure, activate, etc.) | Khi làm việc với hardware drivers |
| **ros-expert** | Debug TF2, Topics, Services, QoS | Khi robot "không chạy" |
| **tdd-ros2** | Test-Driven Development với GTest/Pytest | Khi viết tính năng mới |
| **ros2-build-resolver** | Sửa lỗi CMake, linking, dependency | Khi `colcon build` thất bại |
| **system-architect** | Thiết kế Node graph, QoS, Namespace | Khi setup hệ thống mới |
| **code-review** | Review 6 khía cạnh: Architecture, Security, Performance... | Trước khi merge PR |
| **git-expert** | Giải quyết merge conflict, branching | Khi làm việc nhóm |

### 🔄 Workflows (Quy trình - Slash Commands)
Workflows là các quy trình step-by-step được kích hoạt bằng `/command`.

| Command | Mô tả |
|:--------|:------|
| `/ci-local-pipeline` | Chạy CI/CD cục bộ: rosdep → lint → build → test |
| `/clean-init` | Xóa build artifacts, reset workspace |
| `/install-ros2` | Hướng dẫn cài đặt ROS 2 Humble trên Ubuntu 22.04 |
| `/universal-request` | Phân loại request (CONSULT/BUILD/DEBUG/OPTIMIZE) và xử lý phù hợp |

### 🧰 Tools (Scripts hỗ trợ)
Scripts Python nằm trong `src/tools/`.

| Tool | Mô tả | Cách chạy |
|:-----|:------|:----------|
| **setup_linter.py** | Tạo `setup.cfg` (Flake8) và `.clang-format` | `python3 src/tools/setup_linter.py` |
| **context_provider.py** | MCP Server cung cấp ROS 2 graph info cho Agent | Tự động qua `mcp_servers.json` |

---

## 🔄 CI/CD Pipeline

Repository có sẵn GitHub Actions (`.github/workflows/ros2_ci.yml`):

| Stage | Tool | Mô tả |
|:------|:-----|:------|
| Build | `colcon build` | Biên dịch tất cả packages |
| Lint (ROS 2) | `ament_flake8`, `ament_cpplint`, `ament_pep257` | Code style theo chuẩn ROS |
| Static Analysis | `cppcheck` | Phát hiện bugs C++ tiềm ẩn |
| Type Check | `mypy` | Kiểm tra kiểu dữ liệu Python |

**Trigger:** Push hoặc Pull Request vào branch `main`.

---

## 📄 Templates

Các mẫu tham khảo trong `templates/`:

| Path | Mô tả |
|:-----|:------|
| `templates/launch/example.launch.py` | Launch file mẫu với Standard Node + Lifecycle Node |

---

## 🤝 Đóng Góp

1. Fork repository
2. Tạo branch: `git checkout -b feature/my-feature`
3. Commit changes: `git commit -m "feat: Add my feature"`
4. Push: `git push origin feature/my-feature`
5. Mở Pull Request

---

## 📄 License

Apache-2.0
