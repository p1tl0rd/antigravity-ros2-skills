# Antigravity ROS 2 Skills

🚀 Bộ cấu hình "Agent-First" biến IDE của bạn thành môi trường phát triển Robot chuyên nghiệp với AI Agents.

## 📦 Cấu Trúc Repository

```
antigravity-ros2-skills/
├── .agent/                    # Agent Configuration
│   ├── rules/                 # Quy tắc code style
│   │   ├── ros2-architecture.md
│   │   ├── ros2-cpp-style.md
│   │   ├── ros2-python-style.md
│   │   └── technical-standards.md
│   ├── skills/                # Kỹ năng chuyên biệt
│   │   ├── ros2-build-master/
│   │   ├── ros2-package-scaffolder/
│   │   ├── lifecycle-operator/
│   │   ├── ros-expert/
│   │   ├── code-review/
│   │   └── git-expert/
│   └── workflows/             # Quy trình làm việc
│       ├── ci-local-pipeline.md
│       ├── clean-init.md
│       ├── install-ros2.md
│       └── universal-request.md
├── .github/workflows/         # GitHub Actions CI/CD
│   └── ros2_ci.yml
├── src/tools/                 # Script hỗ trợ
│   ├── setup_linter.py        # Tạo config flake8/clang-format
│   └── context_provider.py    # MCP Server cho ROS 2 graph
├── templates/                 # Mẫu tham khảo
│   └── launch/
├── .clang-format              # C++ format template
├── mcp_servers.json           # MCP Server config
├── requirements.txt           # Python dependencies
└── README.md
```

## 🔧 Cài Đặt

```bash
# 1. Clone vào workspace ROS 2
cd ~/my_ros2_ws
git clone https://github.com/p1tl0rd/antigravity-ros2-skills.git .

# 2. Cài Dependencies
pip install -r requirements.txt

# 3. Khởi tạo Linter Configs
python3 src/tools/setup_linter.py

# 4. Cài ROS 2 Dependencies (trong workspace có code)
rosdep install --from-paths src --ignore-src -r -y

# 5. Restart IDE để load .agent
```

## ✨ Tính Năng

### Rules (Quy tắc)
| Rule | Trigger | Mô tả |
|:-----|:--------|:------|
| `ros2-architecture` | Always | Build type, Component, Dependency |
| `ros2-cpp-style` | `*.cpp`, `*.hpp` | OOP, Naming, Memory Management |
| `ros2-python-style` | `*.py` | PEP 8, Node Structure |
| `technical-standards` | Always | Type Safety, Error Handling |

### Skills (Kỹ năng)
| Skill | Mô tả |
|:------|:------|
| `ros2_package_scaffolder` | Tạo gói ROS 2 chuẩn |
| `ros2_build_master` | Build thông minh với Colcon |
| `lifecycle_operator` | Quản lý Lifecycle Nodes |
| `ros-expert` | Debug TF2, Topics, Services |
| `code-review` | Review 6 khía cạnh chất lượng |
| `git-expert` | Conflict, Branching, Hooks |

### Workflows (Slash Commands)
| Command | Mô tả |
|:--------|:------|
| `/ci-local-pipeline` | CI/CD cục bộ (Lint, Build, Test) |
| `/clean-init` | Reset workspace |
| `/install-ros2` | Cài đặt ROS 2 Humble |
| `/universal-request` | CONSULT/BUILD/DEBUG/OPTIMIZE |

## 🔄 CI/CD Pipeline

Repository có sẵn GitHub Actions:
- **Trigger**: Push/PR to `main`
- **Environment**: `ros:humble-ros-base` container
- **Checks**: 
  - `flake8` + `cpplint` + `pep257` (ROS 2 linters)
  - `cppcheck` (C++ static analysis)
  - `mypy` (Python type checking)

## 📄 License

Apache-2.0
