# IMU Racket Visualization System

A real-time 3D visualization system for squash racket tracking using IMU (Inertial Measurement Unit) data. This project captures orientation data from an Arduino-based IMU sensor and displays a live 3D squash racket moving within a virtual court environment.

## 🏸 Features

### Real-Time 3D Tracking
- **Live Orientation Tracking**: Instant response to physical Arduino/IMU movement
- **Realistic Squash Racket Model**: 3D compound object with handle, shaft, and head
- **Speed-Based Color Coding**: Racket changes color based on swing speed
  - Blue (0-5 m/s): Slow movements  
  - Yellow (5-10 m/s): Medium speed with gradient transitions
  - Red (>10 m/s): Fast swings

### Squash Court Environment  
- **Authentic Court Dimensions**: 8m × 5m singles court with proper proportions
- **Semi-transparent Walls**: Front wall and side walls for spatial reference
- **Service Box Markings**: Official squash court lines and measurements
- **Enhanced Lighting**: Soft shadows with distant and local lighting

### Data Processing Pipeline
- **Serial Reader**: Collects orientation strings from Arduino
- **Smart Parser**: Extracts yaw, pitch, roll values from "Orientation:" lines  
- **Quaternion Conversion**: Transforms Euler angles → quaternion → rotation matrix
- **Trail System**: Yellow trail tracks racket head for drift visualization
- **CSV Logging**: Continuous timestamped data capture
- **Connection Recovery**: Automatic reconnection when Arduino disconnects

### Performance
- **60 FPS Target**: Smooth real-time visualization
- **Frame Rate Control**: Intelligent throttling for consistent performance
- **Debug Monitoring**: Comprehensive logging and error handling

## 🚀 Quick Start

### Prerequisites
- Python 3.7 or higher
- Arduino with IMU sensor (BNO055 recommended)
- USB connection for serial communication

### Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd imu-racket-visualization
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload Arduino code**:
   - Open `arduino/live_imu_sender/live_imu_sender.ino` in Arduino IDE
   - Upload to your Arduino with connected IMU sensor

### Running the Visualization

1. **Connect your Arduino** with IMU sensor via USB

2. **Find your serial port**:
   ```bash
   python check_ports.py
   ```

3. **Run the visualization**:
   ```bash
   python live_racket_debug.py --port /dev/cu.usbmodem1101
   ```

### Command Line Options
```bash
python live_racket_debug.py [OPTIONS]

Options:
  --port TEXT          Serial port for IMU (default: /dev/cu.usbmodem1101)
  --baud INTEGER       Baud rate (default: 115200)
  --length FLOAT       Racket length in meters (default: 0.7)
  --head-radius FLOAT  Racket head radius in meters (default: 0.15)
  --reset             Reset Arduino before connecting
```

## 📁 Project Structure

```
├── live_racket_debug.py      # Main visualization application (debug version)
├── serial_reader.py          # IMU data reader with CSV logging
├── reset_arduino.py          # Arduino reset utility
├── requirements.txt          # Python dependencies
├── README.md                # This file
├── 
├── arduino/
│   └── live_imu_sender/
│       └── live_imu_sender.ino  # Arduino IMU sensor code
│
├── data/                     # Generated CSV data files
│   └── imu_data_*.csv       # Timestamped IMU recordings
│
└── legacy/                   # Previous versions and experimental code
    ├── live_arrow_debug.py   # Original arrow-based visualization
    ├── live_arrow.py         # Clean arrow version
    └── ...                   # Other development files
```

## 🎮 Usage

### Basic Operation
1. **Physical Setup**: Attach the Arduino+IMU to your squash racket handle
2. **Start Visualization**: Run the Python script with your serial port
3. **Live Tracking**: Move the racket and watch the 3D model respond instantly
4. **Speed Feedback**: Observe color changes as you vary swing speed
5. **Drift Monitoring**: Use the yellow trail to check for sensor drift

### Debug Information
The visualization displays real-time debug information including:
- Frame rate and processing statistics  
- Current orientation (yaw, pitch, roll)
- Quaternion values
- Racket head center position
- Instantaneous swing speed
- Color mapping status

### Data Logging
- **Automatic CSV Generation**: All IMU data is logged with timestamps
- **File Rotation**: New files created periodically to manage size
- **Data Format**: Timestamp, yaw, pitch, roll, quaternion components

## 🔧 Technical Details

### Data Flow Architecture
```
Arduino IMU → Serial Port → IMUReader → Parser → Quaternion Converter → 3D Renderer
                                                      ↓
                                              CSV Logger ← Trail System
```

### Key Components

**RacketScene Class**: Manages 3D visualization
- Compound racket object (handle, shaft, head)
- Court environment rendering
- Speed-based color mapping
- Trail system for drift detection

**DebugRacketVisualizer Class**: Core application logic  
- IMU data processing pipeline
- Frame rate management
- Error handling and recovery
- Debug information display

**IMUReader Class**: Serial communication
- Non-blocking data acquisition
- Automatic reconnection
- CSV data logging
- Frame queuing system

### Coordinate System
- **X-axis**: Left/right (red reference arrow)
- **Y-axis**: Up/down (green reference arrow)  
- **Z-axis**: Forward/back (blue reference arrow)
- **Origin**: Player position (racket handle at rest)

## 🛠️ Development

### Adding Features
The modular architecture makes it easy to extend:
- **New Visualizations**: Modify `RacketScene` class
- **Data Processing**: Enhance `IMUReader` or add new parsers
- **UI Elements**: Add information displays or controls
- **Analysis Tools**: Process logged CSV data

### Debug Mode
The current version includes extensive debugging:
- Detailed console output for troubleshooting
- Real-time performance monitoring  
- Error tracking and statistics
- Connection status indicators

### Arduino Customization
Modify `arduino/live_imu_sender/live_imu_sender.ino` to:
- Support different IMU sensors
- Adjust sampling rates
- Change data output format
- Add additional sensor data

## 📊 Performance Tuning

### Optimization Tips
- **Serial Baud Rate**: Higher rates (115200) for faster updates
- **Frame Rate**: Adjust target FPS based on hardware capabilities
- **Trail Length**: Reduce `max_trail_points` for better performance
- **Debug Output**: Disable verbose logging for production use

### System Requirements
- **CPU**: Modern multi-core processor recommended
- **GPU**: Dedicated graphics card helpful for VPython rendering
- **RAM**: 4GB minimum, 8GB recommended
- **USB**: USB 2.0 or higher for reliable serial communication

## 🐛 Troubleshooting

### Common Issues

**Connection Problems**:
```bash
# Reset Arduino if connection fails
python reset_arduino.py --port /dev/cu.usbmodem1101

# Check available ports
python check_ports.py
```

**Performance Issues**:
- Reduce trail length in code
- Lower target FPS
- Close other applications using graphics

**Sensor Drift**:
- Recalibrate IMU sensor
- Check for magnetic interference
- Ensure stable mounting

### Getting Help
- Check debug console output for error messages
- Verify Arduino code is properly uploaded
- Ensure all dependencies are installed correctly
- Test with basic Arduino serial monitor first

## 📈 Future Enhancements

### Planned Features
- **Swing Analysis**: Detailed swing pattern recognition
- **Training Modes**: Specific drills and feedback
- **Multi-Player**: Support for multiple rackets
- **Mobile App**: Smartphone-based visualization
- **VR Integration**: Virtual reality training environment

### Contributing
Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable  
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- VPython for excellent 3D visualization capabilities
- Arduino and IMU sensor communities for hardware support
- Squash community for domain expertise and feedback

---

**Happy Squashing! 🏸** 