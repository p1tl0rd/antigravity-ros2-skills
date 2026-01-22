# Antigravity ROS 2 Skills

🚀 The "Agent-First" configuration that transforms your IDE into a professional Robotics development environment powered by AI Agents (Google Gemini, Claude, Cursor, etc.).

---

## 📦 Installation

### Step 1: Clone into ROS 2 Workspace
```bash
# Create a new workspace (or use an existing one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone the repository into the ROOT of your workspace (not inside /src)
git clone https://github.com/p1tl0rd/antigravity-ros2-skills.git .
```

### Step 2: Install Python Dependencies
```bash
pip install -r requirements.txt
```

### Step 3: Initialize Linter Configs
```bash
python3 src/tools/setup_linter.py
```
> This script generates `setup.cfg` (Flake8) and `.clang-format` (C++) in the root directory.

### Step 4: Install ROS 2 Dependencies
```bash
# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update

# Install dependencies for packages in src/
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Restart IDE
Restart your IDE (Cursor/VSCode) to load the `.agent` directory and MCP Servers.

---

## 🚀 Usage Guide

**Core Principle**: Communicate naturally. The Agent is proactive and will automatically select the appropriate tool for the job.

### 1. Natural Interaction (Recommended)
You don't need to remember Skill or Rule names. Just describe your goal:

*   *"Create a Python package to control a motor via Serial."*
    → Agent automatically activates `ros2_package_scaffolder`.
*   *"This code looks checks, review it for me."*
    → Agent automatically uses `code-review`.
*   *"Build failed, please fix it."*
    → Agent automatically uses `ros2-build-resolver`.

### 2. Automated Rules (Zero-Config)
You don't need to remind the Agent to follow standards. Just write code, and the Agent will automatically:
*   ✅ Apply OOP for Nodes (per `ros2-architecture`).
*   ✅ Format code to PEP 8 / Google Style standards.
*   ✅ Add Docstrings and Type Hinting.

### 3. Power Users (Slash Commands)
If you prefer explicit commands, you can use workflows:
*   `/ci-local-pipeline`: Run the full local CI/CD pipeline.
*   `/clean-init`: Wipe build artifacts and reset the workspace.

---

## 📋 Repository Structure

```
antigravity-ros2-skills/
├── .agent/                    # 🧠 Agent Configuration
│   ├── rules/                 # Code Style Rules (Always active)
│   ├── skills/                # Specialized Capabilities (Context-aware)
│   └── workflows/             # Standard Procedures (Slash commands)
├── .github/workflows/         # 🔄 GitHub Actions CI/CD
├── src/tools/                 # 🛠️ Helper Scripts
├── templates/                 # 📄 Reference Templates
├── mcp_servers.json           # MCP Server Config
├── requirements.txt           # Python Dependencies
└── README.md
```

---

## 📖 Component Details

### 📐 Rules (Always Active)
Rules are automatically enforced based on the active file context.

| Rule | Trigger | Description |
|:-----|:--------|:------------|
| **ros2-architecture** | Global | Segregates Build Types (C++/Python/Interface). Enforces Component-based Design. |
| **ros2-cpp-style** | `*.cpp`, `*.hpp` | Strict OOP. Naming conventions (`snake_case` methods, `member_` vars). No `new/delete`. |
| **ros2-python-style** | `*.py` | PEP 8. Nodes must inherit `rclpy.node.Node`. Standard entry points. |
| **technical-standards** | Global | KISS, DRY, YAGNI. Type Safety. Error Handling. Async best practices. |

### 🛠️ Skills (Specialized Capabilities)
Capabilities that the Agent automatically activates to solve complex tasks.

| Skill | Function | Automatic Activation Context |
|:------|:---------|:-----------------------------|
| **ros2_package_scaffolder** | Create standard ROS 2 packages | When asking to create a new package |
| **ros2_build_master** | Smart build management with Colcon | When running workspace builds |
| **lifecycle_operator** | Manage Lifecycle Nodes | When configuring or testing hardware drivers |
| **ros-expert** | Debug TF2, Topics, Services, QoS | When analyzing runtime/communication issues |
| **tdd-ros2** | Test-Driven Development (GTest/Pytest) | When writing unit tests or new features |
| **ros2-build-resolver** | Fix CMake, linking, dependency errors | When `colcon build` fails |
| **system-architect** | Design Node graphs, QoS, Namespaces | When designing system architecture |
| **code-review** | 6-Aspect Review (Arch, Security, Perf...) | When asking for code/PR review |
| **git-expert** | Resolve merge conflicts, branching | When handling Git operations |

### 🔄 Workflows (Standard Procedures)
Standardized procedures ensuring consistency. Can be triggered via Slash Commands.

| Command | Function |
|:--------|:---------|
| `/ci-local-pipeline` | Run local CI/CD: rosdep → lint → build → test |
| `/clean-init` | Clean build artifacts, reset workspace |
| `/install-ros2` | Install ROS 2 Humble on Ubuntu 22.04 |
| `/universal-request` | Standard request handling (CONSULT → BUILD → DEBUG) |

### 🧰 Tools (Helper Scripts)
Python scripts located in `src/tools/`.

| Tool | Description | Usage |
|:-----|:------------|:------|
| **setup_linter.py** | Generates `setup.cfg` & `.clang-format` | `python3 src/tools/setup_linter.py` |
| **context_provider.py** | MCP Server for ROS 2 graph info | Automated via `mcp_servers.json` |

---

## 🔄 CI/CD Pipeline

Includes GitHub Actions configuration (`.github/workflows/ros2_ci.yml`):

| Stage | Tool | Description |
|:------|:-----|:------------|
| Build | `colcon build` | Compiles all packages |
| Lint (ROS 2) | `ament_flake8`, `ament_cpplint` | Checks code against ROS standards |
| Static Analysis | `cppcheck` | Detects C++ bugs and performance issues |
| Type Check | `mypy` | Verifies Python type safety |

**Trigger:** Push or Pull Request to `main`.

---

## 📄 Templates

Reference examples in `templates/`:

| Path | Description |
|:-----|:------------|
| `templates/launch/example.launch.py` | Launch file with Standard Node + Lifecycle Node pattern |

---

## 🤝 Contributing

1. Fork the repository
2. Create your branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m "feat: Add my feature"`
4. Push to the branch: `git push origin feature/my-feature`
5. Open a Pull Request

---

## 📄 License

Apache-2.0
