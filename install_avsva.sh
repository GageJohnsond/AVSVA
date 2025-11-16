#!/bin/bash
# AVSVA Installation and Setup Script
# Texas Tech University - Raider Security

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "=================================================="
echo "  AVSVA Installation Script"
echo "  Autonomous Vehicle Simulation and"
echo "  Vulnerability Analyzer"
echo "=================================================="
echo -e "${NC}"

# Check if running on Ubuntu/Debian
if [ ! -f /etc/os-release ]; then
    echo -e "${RED}Error: Cannot detect OS. This script requires Ubuntu/Debian.${NC}"
    exit 1
fi

source /etc/os-release
if [[ "$ID" != "ubuntu" && "$ID" != "debian" ]]; then
    echo -e "${YELLOW}Warning: This script is designed for Ubuntu/Debian. Your OS: $ID${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}Step 1: Checking ROS Installation${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS environment not sourced. Checking for ROS installation...${NC}"
    
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        echo -e "${GREEN}Found ROS Noetic${NC}"
        source /opt/ros/noetic/setup.bash
        ROS_DISTRO="noetic"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        echo -e "${GREEN}Found ROS Melodic${NC}"
        source /opt/ros/melodic/setup.bash
        ROS_DISTRO="melodic"
    else
        echo -e "${RED}ROS not found. Please install ROS first.${NC}"
        echo "Visit: http://wiki.ros.org/noetic/Installation/Ubuntu"
        exit 1
    fi
else
    echo -e "${GREEN}ROS $ROS_DISTRO detected${NC}"
fi

echo -e "${GREEN}Step 2: Installing ROS Dependencies${NC}"
echo "This may require sudo privileges..."

# Install Husky packages
sudo apt-get update
echo -e "${YELLOW}Installing Husky simulator packages...${NC}"
sudo apt-get install -y ros-$ROS_DISTRO-husky-simulator \
                        ros-$ROS_DISTRO-husky-viz \
                        ros-$ROS_DISTRO-husky-desktop \
                        ros-$ROS_DISTRO-gazebo-ros \
                        ros-$ROS_DISTRO-rviz

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to install ROS packages${NC}"
    exit 1
fi

echo -e "${GREEN}Step 3: Installing Python Dependencies${NC}"

# Check for pip3
if ! command -v pip3 &> /dev/null; then
    echo -e "${YELLOW}pip3 not found. Installing...${NC}"
    sudo apt-get install -y python3-pip
fi

# Install PyQt5
echo -e "${YELLOW}Installing PyQt5...${NC}"
pip3 install PyQt5 --user

# Install other Python dependencies
pip3 install rospkg --user

echo -e "${GREEN}Step 4: Setting up AVSVA${NC}"

# Create directory structure
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "Current directory: $SCRIPT_DIR"

# Create recorded_bags directory
mkdir -p recorded_bags
echo -e "${GREEN}Created recorded_bags directory${NC}"

# Make scripts executable
echo -e "${YELLOW}Making scripts executable...${NC}"
chmod +x avsva_app.py 2>/dev/null || echo "avsva_app.py not found (will need to be made executable)"
chmod +x simulation_scripts/launch_husky_auto_drive.sh
chmod +x simulation_scripts/auto_drive_forward.py
chmod +x attack_scripts/attack_*.py 2>/dev/null

echo -e "${GREEN}Step 5: Setting up ROS Environment${NC}"

# Add ROS source to bashrc if not already there
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}Added ROS source to ~/.bashrc${NC}"
fi

# Create a launcher script
cat > run_avsva.sh << 'EOF'
#!/bin/bash
# AVSVA Launcher Script

# Source ROS
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Run AVSVA
python3 avsva_app.py
EOF

chmod +x run_avsva.sh

echo -e "${GREEN}"
echo "=================================================="
echo "  Installation Complete!"
echo "=================================================="
echo -e "${NC}"

echo -e "${BLUE}To run AVSVA:${NC}"
echo "  1. Open a terminal"
echo "  2. cd $SCRIPT_DIR"
echo "  3. ./run_avsva.sh"
echo ""
echo -e "${BLUE}Or directly:${NC}"
echo "  python3 $SCRIPT_DIR/avsva_app.py"
echo ""
echo -e "${YELLOW}Important:${NC}"
echo "  - Make sure to source ROS in each new terminal"
echo "  - Use: source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  - Or restart your terminal for .bashrc changes to take effect"
echo ""
echo -e "${GREEN}Files installed in: $SCRIPT_DIR${NC}"
echo ""
echo -e "${BLUE}Quick Test:${NC}"
echo "  1. Run: ./run_avsva.sh"
echo "  2. Click 'Start Simulation' in the app"
echo "  3. Wait for Gazebo and RViz to load"
echo "  4. Try injecting a vulnerability"
echo ""
echo -e "${YELLOW}For detailed usage, see README.md${NC}"
