# Antigravity ROS 2 Skills Setup

Bộ cấu hình "Agent-First" biến IDE của bạn thành một môi trường phát triển Robot chuyên nghiệp với Google Antigravity.

## Installation

1.  **Clone repository này vào thư mục gốc của ROS 2 Workspace:**
    ```bash
    cd ~/my_ros2_ws
    git clone https://github.com/p1tl0rd/antigravity-ros2-skills.git .
    ```

2.  **Cài đặt Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Khởi tạo Linter Configs:**
    ```bash
    python3 src/tools/setup_linter.py
    ```
    *(Lệnh này sẽ tạo ra `setup.cfg` và `.clang-format` chuẩn).*

4.  **Cài đặt ROS 2 Dependencies:**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

5.  **Restart IDE:**
    Khởi động lại Antigravity/Cursor để IDE nhận diện `.agent` và MCP Servers.

## Features Included
*   **Rules**: Quy tắc kiến trúc (Architecture), Code Style (C++/Python).
*   **Skills**: 
    *   `ros2_package_scaffolder`: Tạo gói chuẩn.
    *   `ros2_build_master`: Build thông minh.
    *   `lifecycle_operator`: Quản lý node.
*   **Workflows**: CI/CD local, Clean Workspace.
*   **MCP Servers**: ROS 2 Context Provider (Node graph realtime).
