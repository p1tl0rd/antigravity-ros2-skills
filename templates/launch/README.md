# Launch File Templates

Thư mục này chứa các mẫu Launch File chuẩn ROS 2 để tham khảo.

## Cách sử dụng

1. Copy file template vào thư mục `launch/` của package bạn.
2. Sửa `package`, `executable`, `name` phù hợp.
3. Thêm vào `setup.py` (Python) hoặc `CMakeLists.txt` (C++).

## Files

| File | Mô tả |
|:-----|:------|
| `example.launch.py` | Mẫu với Standard Node và Lifecycle Node |

## Thêm Launch vào Package

### Python Package (`setup.py`)
```python
data_files=[
    # ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
```

### C++ Package (`CMakeLists.txt`)
```cmake
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```
