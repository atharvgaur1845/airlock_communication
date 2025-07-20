## Project Structure and Components

```
airlock_communication/
├── airlock_communication_pkg/      # Main ROS2 package
│   ├── airlock_communication/     
│   │   ├── __init__.py
│   │   └── airlock_comm_node.py    # Core ROS2 node implementation
│   ├── resource/
│   ├── test/                      # Test files
│   ├── package.xml                # Package manifest
│   └── setup.py                   # Package setup
├── husarnet_firmware/             # ESP32 Husarnet communication
│   └── husarnet_firmware.ino      # ESP32 networking code
└── autonomous_approach/           # Standalone airlock controller
    └── autonomous_airlock.ino     # Arduino-based controller logic
```

## Code Components Explanation

### 1. ROS2 Package (`airlock_communication_pkg/`)

#### Main Node ([`airlock_comm_node.py`](airlock_communication_pkg/airlock_communication/airlock_comm_node.py))
The core ROS2 node that handles:
- Robot zone detection and position tracking
- Communication with ESP32 controller via HTTP/JSON
- Airlock passage sequencing and requests
- Safety monitoring and fallback modes

**Key Features:**
- Zone-based position tracking
- Automatic sequence management
- Heartbeat monitoring
- Manual override support
- Fallback mode for communication failures

**ROS2 Topics:**
- Publishers:
  - `/airlock/status`: Current airlock status updates
  - `/cmd_vel`: Robot velocity commands
  - `/robot/zone`: Zone position updates
- Subscribers:
  - `/robot/pose`: Robot position data
  - `/scan`: LIDAR scan information
  - `/airlock/manual_request`: Manual control inputs

#### Package Files
- `package.xml`: Package manifest with dependencies
- `setup.py`: Python package configuration
- `setup.cfg`: Additional setup configuration
- `resource/`: Package resources
- `test/`: Automated tests
  - `test_copyright.py`: Copyright checks
  - `test_flake8.py`: Code style verification
  - `test_pep257.py`: Docstring validation

### 2. ESP32 Controller (`husarnet_firmware/`)

#### Husarnet Firmware ([`husarnet_firmware.ino`](husarnet_firmware/husarnet_firmware.ino))
Manages network communication using Husarnet protocol:
- HTTP server implementation
- Airlock status management
- Network configuration
- Hardware interfacing

**HTTP Endpoints:**
- `/airlock/request`: Handle passage requests
- `/airlock/status`: Provide current status
- `/airlock/heartbeat`: Connection monitoring
- `/airlock/override`: Manual control interface

### 3. Autonomous Controller (`autonomous_approach/`)

#### Airlock Controller ([`autonomous_airlock.ino`](autonomous_approach/autonomous_airlock.ino))
Standalone control system featuring:
- State machine for airlock sequencing
- Safety monitoring
- Presence detection
- LED status indicators

**Key Functions:**
- Gate control logic
- Safety interlocks
- Emergency stop handling
- Status indication
- Sensor integration