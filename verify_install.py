#!/usr/bin/env python3
"""
Installation Verification Script
Tests that all components are properly installed and can be imported
"""

import sys

print("=" * 60)
print("AV Simulator - Installation Verification")
print("=" * 60)
print()

# Check Python version
print("✓ Checking Python version...")
version = sys.version_info
if version.major >= 3 and version.minor >= 8:
    print(f"  ✓ Python {version.major}.{version.minor}.{version.micro} - OK")
else:
    print(f"  ✗ Python {version.major}.{version.minor}.{version.micro} - FAILED")
    print("  ! Python 3.8 or later required")
    sys.exit(1)

# Check required packages
print("\n✓ Checking required packages...")

packages = {
    'PyQt5': 'PyQt5',
    'matplotlib': 'matplotlib', 
    'seaborn': 'seaborn',
    'pandas': 'pandas',
    'numpy': 'numpy',
    'scipy': 'scipy'
}

missing = []
for display_name, import_name in packages.items():
    try:
        __import__(import_name)
        print(f"  ✓ {display_name} - OK")
    except ImportError:
        print(f"  ✗ {display_name} - MISSING")
        missing.append(import_name)

if missing:
    print(f"\n✗ Missing packages: {', '.join(missing)}")
    print("\nTo install missing packages, run:")
    print(f"  pip install {' '.join(missing)}")
    sys.exit(1)

# Check application modules
print("\n✓ Checking application modules...")

modules = [
    'bag_player',
    'data_analyzer',
    'vulnerability_injector',
    'simulator',
    'visualizer',
    'report_generator',
    'config'
]

for module in modules:
    try:
        __import__(module)
        print(f"  ✓ {module}.py - OK")
    except ImportError as e:
        print(f"  ✗ {module}.py - FAILED")
        print(f"     Error: {e}")
        sys.exit(1)

# Check main application
print("\n✓ Checking main application...")
try:
    import av_simulator
    print("  ✓ av_simulator.py - OK")
except ImportError as e:
    print("  ✗ av_simulator.py - FAILED")
    print(f"     Error: {e}")
    sys.exit(1)

# All checks passed
print("\n" + "=" * 60)
print("✓ All checks passed!")
print("=" * 60)
print("\nYour installation is complete and ready to use.")
print("\nTo start the application, run:")
print("  python av_simulator.py")
print("\nOr use the launcher:")
print("  ./run_simulator.sh")
print()
