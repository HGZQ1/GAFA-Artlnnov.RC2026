#!/usr/bin/env python3
"""
Docker环境测试脚本
用于验证所有依赖是否正确安装
"""

import sys
import subprocess

def print_section(title):
    """打印分节标题"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def check_import(module_name, display_name=None):
    """检查模块是否可以导入"""
    if display_name is None:
        display_name = module_name
    
    try:
        __import__(module_name)
        print(f"✅ {display_name}: OK")
        return True
    except ImportError as e:
        print(f"❌ {display_name}: FAILED - {e}")
        return False

def check_pytorch():
    """检查PyTorch和CUDA"""
    print_section("PyTorch和CUDA检查")
    
    try:
        import torch
        print(f"✅ PyTorch版本: {torch.__version__}")
        print(f"✅ CUDA可用: {torch.cuda.is_available()}")
        
        if torch.cuda.is_available():
            print(f"✅ CUDA版本: {torch.version.cuda}")
            print(f"✅ GPU数量: {torch.cuda.device_count()}")
            for i in range(torch.cuda.device_count()):
                print(f"   GPU {i}: {torch.cuda.get_device_name(i)}")
        else:
            print("⚠️  CUDA不可用（CPU模式）")
        
        return True
    except Exception as e:
        print(f"❌ PyTorch检查失败: {e}")
        return False

def check_realsense():
    """检查RealSense SDK"""
    print_section("RealSense SDK检查")
    
    try:
        import pyrealsense2 as rs
        print(f"✅ pyrealsense2版本: {rs.__version__}")
        
        # 尝试枚举设备
        ctx = rs.context()
        devices = ctx.query_devices()
        device_count = len(devices)
        
        print(f"✅ 检测到 {device_count} 个RealSense设备")
        
        for i, dev in enumerate(devices):
            print(f"   设备 {i}:")
            print(f"     名称: {dev.get_info(rs.camera_info.name)}")
            print(f"     序列号: {dev.get_info(rs.camera_info.serial_number)}")
            print(f"     固件版本: {dev.get_info(rs.camera_info.firmware_version)}")
        
        if device_count == 0:
            print("⚠️  未检测到RealSense设备，请检查相机连接")
        
        return True
    except Exception as e:
        print(f"❌ RealSense检查失败: {e}")
        return False

def check_yolo():
    """检查YOLO模型"""
    print_section("YOLO模型检查")
    
    try:
        from ultralytics import YOLO
        print("✅ Ultralytics YOLO已安装")
        
        # 检查模型文件
        import os
        weights_dir = "/workspace/weights"
        if os.path.exists(weights_dir):
            weights_files = [f for f in os.listdir(weights_dir) if f.endswith('.pt')]
            print(f"✅ 权重目录: {weights_dir}")
            print(f"   找到 {len(weights_files)} 个权重文件:")
            for w in weights_files:
                size = os.path.getsize(os.path.join(weights_dir, w)) / 1024 / 1024
                print(f"     - {w} ({size:.2f} MB)")
        else:
            print(f"⚠️  权重目录不存在: {weights_dir}")
        
        return True
    except Exception as e:
        print(f"❌ YOLO检查失败: {e}")
        return False

def check_opencv():
    """检查OpenCV"""
    print_section("OpenCV检查")
    
    try:
        import cv2
        print(f"✅ OpenCV版本: {cv2.__version__}")
        
        # 检查GUI支持
        try:
            # 测试创建窗口（不显示）
            test_img = cv2.imread('/workspace/data/images/bus.jpg')
            if test_img is not None:
                print("✅ 图像读取功能正常")
            else:
                print("⚠️  测试图像不存在，但OpenCV功能正常")
        except Exception as e:
            print(f"⚠️  GUI测试警告: {e}")
        
        return True
    except Exception as e:
        print(f"❌ OpenCV检查失败: {e}")
        return False

def check_system_info():
    """显示系统信息"""
    print_section("系统信息")
    
    # Python版本
    print(f"Python版本: {sys.version}")
    
    # 操作系统
    import platform
    print(f"操作系统: {platform.system()} {platform.release()}")
    print(f"架构: {platform.machine()}")
    
    # CPU信息
    try:
        import psutil
        print(f"CPU核心数: {psutil.cpu_count()}")
        print(f"内存总量: {psutil.virtual_memory().total / 1024**3:.2f} GB")
    except ImportError:
        print("⚠️  psutil未安装，无法获取详细系统信息")
    
    # 检查CUDA工具包
    try:
        result = subprocess.run(['nvcc', '--version'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ CUDA Toolkit已安装")
            print(f"   {result.stdout.split('release')[1].split(',')[0].strip()}")
    except:
        print("⚠️  CUDA Toolkit未安装或不在PATH中")

def check_all_dependencies():
    """检查所有Python依赖"""
    print_section("Python依赖检查")
    
    dependencies = {
        'numpy': 'NumPy',
        'pandas': 'Pandas',
        'opencv-python': 'OpenCV (cv2)',
        'pillow': 'Pillow (PIL)',
        'matplotlib': 'Matplotlib',
        'scipy': 'SciPy',
        'tqdm': 'tqdm',
        'pyyaml': 'PyYAML',
    }
    
    success_count = 0
    total_count = len(dependencies)
    
    for module, name in dependencies.items():
        # 处理特殊情况
        if module == 'opencv-python':
            module_to_import = 'cv2'
        elif module == 'pillow':
            module_to_import = 'PIL'
        elif module == 'pyyaml':
            module_to_import = 'yaml'
        else:
            module_to_import = module
        
        if check_import(module_to_import, name):
            success_count += 1
    
    print(f"\n总计: {success_count}/{total_count} 个依赖项正常")
    return success_count == total_count

def test_simple_detection():
    """执行简单的检测测试"""
    print_section("简单检测测试")
    
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
        
        # 创建测试图像
        test_image = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.rectangle(test_image, (100, 100), (300, 300), (255, 255, 255), -1)
        
        print("创建测试图像...")
        
        # 尝试加载模型
        try:
            model = YOLO('yolov8n.pt')
            print("✅ YOLOv8n模型加载成功")
            
            # 执行推理
            results = model(test_image, verbose=False)
            print("✅ 推理执行成功")
            print(f"   检测到 {len(results[0].boxes)} 个目标")
            
            return True
        except Exception as e:
            print(f"⚠️  模型测试失败（可能需要下载权重）: {e}")
            return False
            
    except Exception as e:
        print(f"❌ 检测测试失败: {e}")
        return False

def main():
    """主函数"""
    print("\n" + "🔍 Docker环境测试".center(60, "="))
    print("此脚本将验证所有必要的依赖和配置\n")
    
    results = []
    
    # 系统信息
    check_system_info()
    
    # 核心依赖检查
    results.append(("PyTorch", check_pytorch()))
    results.append(("OpenCV", check_opencv()))
    results.append(("RealSense", check_realsense()))
    results.append(("YOLO", check_yolo()))
    results.append(("Python依赖", check_all_dependencies()))
    
    # 功能测试
    results.append(("检测测试", test_simple_detection()))
    
    # 总结
    print_section("测试总结")
    
    success = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{name}: {status}")
    
    print(f"\n总计: {success}/{total} 项测试通过")
    
    if success == total:
        print("\n🎉 所有测试通过！环境配置正确。")
        return 0
    else:
        print("\n⚠️  部分测试失败，请检查配置。")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
