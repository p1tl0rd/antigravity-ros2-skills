# Antigravity ROS 2 Skills

🚀 Bộ cấu hình "Agent-First" biến IDE của bạn thành môi trường phát triển Robot chuyên nghiệp với AI Agents.

## 📦 Nội Dung

```
.agent/
├── rules/           # Quy tắc code style (C++, Python, Technical)
├── skills/          # Kỹ năng chuyên biệt (ROS Expert, Code Review, Git...)
└── workflows/       # Quy trình làm việc chuẩn (CI, Clean Init, Universal...)

src/tools/           # Script hỗ trợ (Linter setup, Context Provider)
.github/workflows/   # GitHub Actions CI/CD
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

# 5. Restart IDE
```

## ✨ Tính Năng

### Rules (Quy tắc)
| Rule | Mô tả |
|:-----|:------|
| `ros2-architecture` | Build type, Component, Dependency |
| `ros2-cpp-style` | OOP, Naming, Memory Management |
| `ros2-python-style` | PEP 8, Node Structure, Entry Points |
| `technical-standards` | Type Safety, Error Handling (Language-agnostic) |

### Skills (Kỹ năng)
| Skill | Mô tả |
|:------|:------|
| `ros2_package_scaffolder` | Tạo gói ROS 2 chuẩn |
| `ros2_build_master` | Build thông minh với Colcon |
| `lifecycle_operator` | Quản lý Lifecycle Nodes |
| `ros-expert` | Debug TF2, Topics, Services |
| `code-review` | Review 6 khía cạnh chất lượng |
| `git-expert` | Conflict, Branching, Hooks |

### Workflows (Quy trình)
| Workflow | Mô tả |
|:---------|:------|
| `/ci-local-pipeline` | CI/CD cục bộ (Lint, Build, Test) |
| `/clean-init` | Reset workspace |
| `/install-ros2` | Cài đặt ROS 2 Humble |
| `/universal-request` | CONSULT/BUILD/DEBUG/OPTIMIZE |

## 🔄 CI/CD

Repository này có sẵn GitHub Actions CI:
- **Trigger**: Push/PR to `main`
- **Tests**: `flake8`, `cpplint`, `pep257`
- **Container**: `ros:humble-ros-base`

## 📄 License

Apache-2.0
