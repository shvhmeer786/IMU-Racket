import serial
import threading
import queue
import time
import csv
import os
import json
import logging
import re
from datetime import datetime, timedelta
from typing import Optional, Dict, Tuple, Any

# Configure logging
logging.basicConfig(level=logging.INFO)  # Reduce debug noise
logger = logging.getLogger(__name__)

class IMUReader:
    """
    Non-blocking IMU data reader with CSV auto-rotation and auto-reconnection.
    
    Reads IMU data from serial port, emits frames via Queue, and auto-rotates CSV files.
    Handles USB reconnection automatically.
    """
    
    def __init__(self, port: str, baud: int = 115200, chunk_minutes: int = 10):
        """
        Initialize IMU reader.
        
        Args:
            port: Serial port (e.g., '/dev/cu.usbmodem1101')
            baud: Baud rate (default: 115200)
            chunk_minutes: Minutes per CSV file chunk
        """
        self.port = port
        self.baud = baud
        self.chunk_minutes = chunk_minutes
        
        # Threading and state control
        self.serial_conn = None
        self.read_thread = None
        self._running = False
        self._stop_event = threading.Event()
        
        # Data queue (using frame_queue but also providing data_queue alias for compatibility)
        self.frame_queue = queue.Queue(maxsize=1000)
        self.data_queue = self.frame_queue  # Alias for backward compatibility
        
        # CSV file handling
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None
        self.current_csv_start = None
        
        # Connection management - LESS AGGRESSIVE
        self.reconnect_attempt_count = 0
        self.max_reconnect_attempts = 3  # Reduced from higher number
        self.last_reconnect_attempt = 0
        self.reconnect_delay = 1.0  # Reduced from higher value
        self.reset_count = 0
        self.max_resets_per_minute = 2  # Limit resets
        self.last_reset_time = 0
        
        # Data processing
        self.frame_count = 0
        self.successful_frames = 0
        self.last_frame_time = 0
        
        # Regex patterns for cleaning serial data
        self.control_char_pattern = re.compile(r'[\x00-\x1f\x7f-\x9f]')
        self.ansi_escape_pattern = re.compile(r'\x1b\[[0-9;]*[mGKHF]')
        
        # Current sensor data buffer
        self.current_sensor_data = {}
        
        logger.info(f"IMUReader initialized for port {port} at {baud} baud")
    
    def start(self):
        """Start the IMU reader thread."""
        if self._running:
            logger.warning("IMU reader already running")
            return
        
        self._running = True
        self._stop_event.clear()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        logger.info("Read loop started")
        
    def stop(self):
        """Stop the IMU reader thread."""
        if not self._running:
            return
        
        logger.info("Stopping IMUReader...")
        self._running = False
        self._stop_event.set()
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        self._close_serial()
        self._close_csv()
        logger.info("IMUReader stopped")
    
    def get_frame(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """Get next frame from queue."""
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def _read_loop(self):
        """Main read loop - LESS AGGRESSIVE RECONNECTION."""
        logger.info("Read loop started")
        
        while self._running:
            try:
                # Try to maintain connection with less aggressive approach
                if not self._ensure_connection():
                    time.sleep(0.5)  # Brief pause before retrying
                    continue
                
                # Ensure CSV file is ready
                self._ensure_csv_file()
                
                # Read data with better error handling
                if self._read_data():
                    self.reconnect_attempt_count = 0  # Reset on success
                else:
                    time.sleep(0.1)  # Brief pause on read failure
                    
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                time.sleep(0.5)
        
        logger.info("Read loop ended")
    
    def _ensure_connection(self) -> bool:
        """Ensure serial connection is available - LESS AGGRESSIVE."""
        # Check if connection is already good
        if self.serial_conn:
            try:
                # Quick check - don't be too aggressive
                if self.serial_conn.in_waiting >= 0:
                    return True
            except Exception:
                # Connection is broken, close it
                self._close_serial()
        
        # Throttle reconnection attempts
        current_time = time.time()
        if current_time - self.last_reconnect_attempt < self.reconnect_delay:
            return False
        
        self.last_reconnect_attempt = current_time
        self.reconnect_attempt_count += 1
        
        try:
            if self.serial_conn:
                self.serial_conn.close()
                time.sleep(0.1)  # Brief pause before reconnecting
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0,  # Reasonable timeout
                write_timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # ONLY reset if we haven't reset recently and it's really needed
            should_reset = (
                self.reconnect_attempt_count > 2 and  # Only after multiple failures
                current_time - self.last_reset_time > 30 and  # Not too recently
                self.reset_count < self.max_resets_per_minute  # Not too many resets
            )
            
            if should_reset:
                logger.info("Resetting Arduino (limited reset)...")
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                time.sleep(0.1)
                self.serial_conn.dtr = True
                self.serial_conn.rts = True
                time.sleep(0.1)
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                time.sleep(2.0)  # Wait for boot
                self.reset_count += 1
                self.last_reset_time = current_time
            else:
                # Just clear buffers, don't reset
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                time.sleep(0.5)  # Brief pause
            
            logger.info(f"Connected to {self.port} (attempt {self.reconnect_attempt_count})")
            self.reconnect_attempt_count = 0
            return True
            
        except Exception as e:
            if self.reconnect_attempt_count <= self.max_reconnect_attempts:
                logger.debug(f"Connection failed (attempt {self.reconnect_attempt_count}): {e}")
            else:
                logger.warning(f"Connection failed after {self.max_reconnect_attempts} attempts: {e}")
                # Exponential backoff after max attempts
                self.reconnect_delay = min(self.reconnect_delay * 1.2, 5.0)  # Less aggressive backoff
            
            self.serial_conn = None
            return False
    
    def _close_serial(self):
        """Close serial connection."""
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception as e:
                logger.debug(f"Error closing serial: {e}")
            finally:
                self.serial_conn = None
    
    def _ensure_csv_file(self):
        """Ensure CSV file is open and current."""
        current_time = datetime.now()
        
        # Check if we need to rotate the file
        if (self.current_csv_start is None or 
            current_time >= self.current_csv_start + timedelta(minutes=self.chunk_minutes)):
            
            self._close_csv()
            self._open_new_csv(current_time)
    
    def _open_new_csv(self, timestamp: datetime):
        """Open a new CSV file with timestamp."""
        try:
            # Generate filename with timestamp
            timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S")
            self.csv_filename = f"imu_data_{timestamp_str}.csv"
            
            # Create directory if it doesn't exist
            os.makedirs("data", exist_ok=True)
            filepath = os.path.join("data", self.csv_filename)
            
            # Open CSV file
            self.csv_file = open(filepath, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            self.csv_writer.writerow([
                'timestamp', 'heading', 'roll', 'pitch', 
                'acc_x', 'acc_y', 'acc_z',
                'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
                'quat_w', 'quat_x', 'quat_y', 'quat_z',
                'mag_x', 'mag_y', 'mag_z',
                'temperature'
            ])
            
            self.current_csv_start = timestamp
            logger.info(f"Opened new CSV file: {filepath}")
            
        except Exception as e:
            logger.error(f"Failed to open CSV file: {e}")
    
    def _close_csv(self):
        """Close current CSV file."""
        if self.csv_file:
            try:
                self.csv_file.close()
                logger.debug(f"Closed CSV file: {self.csv_filename}")
            except Exception as e:
                logger.error(f"Error closing CSV file: {e}")
            finally:
                self.csv_file = None
                self.csv_writer = None
    
    def _read_data(self) -> bool:
        """Read and process data from serial connection."""
        if not self.serial_conn:
            return False
        
        try:
            # Check if data is available
            if self.serial_conn.in_waiting == 0:
                return True  # No data, but connection is fine
            
            # Read available data
            raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
            if not raw_data:
                return True  # No data but connection OK
            
            # Decode with error handling
            try:
                text_data = raw_data.decode('utf-8', errors='ignore')
            except UnicodeDecodeError:
                text_data = raw_data.decode('ascii', errors='ignore')
            
            # Process each line
            lines = text_data.strip().split('\n')
            for line in lines:
                if line.strip():
                    self._process_line(line.strip())
            
            return True
            
        except serial.SerialException as e:
            logger.warning(f"Serial I/O error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error reading data: {e}")
            return False
    
    def _process_line(self, line: str):
        """Process a single line of data."""
        try:
            # Clean the line
            cleaned_line = self._clean_line(line)
            if not cleaned_line:
                return
            
            logger.debug(f"Cleaned line: '{cleaned_line}'")
            
            # Try to parse sensor data
            frame = self._parse_sensor_line(cleaned_line)
            if frame:
                self.successful_frames += 1
                self.last_frame_time = time.time()
                
                # Add to queue
                try:
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    # If queue is full, remove old item and add new one
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait(frame)
                    except queue.Empty:
                        pass
                
                # Write to CSV
                if self.csv_writer:
                    self._write_csv_row(frame)
                    
        except Exception as e:
            logger.debug(f"Error processing line '{line}': {e}")
    
    def _clean_line(self, line: str) -> str:
        """Clean a line of serial data."""
        # Remove control characters and ANSI escape sequences
        cleaned = self.ansi_escape_pattern.sub('', line)
        cleaned = self.control_char_pattern.sub('', cleaned)
        
        # Remove common CircuitPython artifacts
        cleaned = cleaned.replace('\r', '').replace('\n', '').strip()
        
        return cleaned
    
    def _parse_sensor_line(self, line: str) -> Optional[Dict[str, Any]]:
        """Parse sensor data from a line and accumulate complete frames."""
        try:
            # Parse orientation data
            if 'Orientation:' in line:
                parts = line.split('Orientation:')
                if len(parts) >= 2:
                    orientation_part = parts[-1].strip()
                    values = [float(x.strip()) for x in orientation_part.split(',')]
                    if len(values) >= 3:
                        heading, roll, pitch = values[0], values[1], values[2]
                        self.current_sensor_data['euler'] = (heading, roll, pitch)
                        logger.debug(f"Parsed orientation: heading={heading}, roll={roll}, pitch={pitch}")
            
            # Parse raw acceleration data
            elif 'Accel:' in line:
                parts = line.split('Accel:')
                if len(parts) >= 2:
                    accel_part = parts[-1].strip()
                    values = [float(x.strip()) for x in accel_part.split(',')]
                    if len(values) >= 3:
                        ax, ay, az = values[0], values[1], values[2]
                        self.current_sensor_data['acc'] = (ax, ay, az)
                        logger.debug(f"Parsed acceleration: ax={ax}, ay={ay}, az={az}")
            
            # Parse linear acceleration data
            elif 'LinAccel:' in line:
                parts = line.split('LinAccel:')
                if len(parts) >= 2:
                    lin_accel_part = parts[-1].strip()
                    values = [float(x.strip()) for x in lin_accel_part.split(',')]
                    if len(values) >= 3:
                        lax, lay, laz = values[0], values[1], values[2]
                        self.current_sensor_data['lin_acc'] = (lax, lay, laz)
                        logger.debug(f"Parsed linear acceleration: lax={lax}, lay={lay}, laz={laz}")
            
            # Parse temperature data
            elif 'Temp:' in line:
                parts = line.split('Temp:')
                if len(parts) >= 2:
                    temp_part = parts[-1].strip()
                    temp = float(temp_part)
                    self.current_sensor_data['temp'] = temp
                    logger.debug(f"Parsed temperature: temp={temp}°C")
            
            # Check if we have enough data to create a frame
            if 'euler' in self.current_sensor_data:
                # Create frame with available data
                frame = {
                    'ts': int(time.time() * 1000),  # Timestamp in milliseconds
                    'quat': (1.0, 0.0, 0.0, 0.0),  # Dummy quaternion
                    'acc': self.current_sensor_data.get('acc', (0.0, 0.0, 9.81)),
                    'lin_acc': self.current_sensor_data.get('lin_acc', (0.0, 0.0, 0.0)),
                    'mag': (0.0, 0.0, 0.0),        # Dummy magnetometer
                    'euler': self.current_sensor_data['euler'],
                    'temp': self.current_sensor_data.get('temp', 0.0)
                }
                
                # Clear the buffer after creating frame
                self.current_sensor_data.clear()
                
                logger.debug(f"Successfully parsed complete frame: {frame}")
                return frame
                
        except (ValueError, IndexError) as e:
            logger.debug(f"Failed to parse sensor line '{line}': {e}")
        
        return None
    
    def _write_csv_row(self, frame: Dict[str, Any]):
        """Write frame data to CSV."""
        if not self.csv_writer:
            return
        
        try:
            heading, roll, pitch = frame['euler']
            acc_x, acc_y, acc_z = frame['acc']
            lin_acc_x, lin_acc_y, lin_acc_z = frame['lin_acc']
            quat_w, quat_x, quat_y, quat_z = frame['quat']
            mag_x, mag_y, mag_z = frame['mag']
            temp = frame['temp']
            
            self.csv_writer.writerow([
                frame['ts'], heading, roll, pitch,
                acc_x, acc_y, acc_z,
                lin_acc_x, lin_acc_y, lin_acc_z,
                quat_w, quat_x, quat_y, quat_z,
                mag_x, mag_y, mag_z,
                temp
            ])
            
            # Flush periodically
            if self.frame_count % 10 == 0:
                self.csv_file.flush()
                
        except Exception as e:
            logger.error(f"Error writing CSV row: {e}")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get reader statistics."""
        return {
            'frames_total': self.frame_count,
            'frames_successful': self.successful_frames,
            'reconnect_attempts': self.reconnect_attempt_count,
            'connected': self.serial_conn is not None,
            'last_frame_time': self.last_frame_time,
            'csv_filename': self.csv_filename
        }
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        
    @property
    def is_connected(self) -> bool:
        """Check if serial connection is active."""
        return self.serial_conn is not None
    
    @property
    def queue_size(self) -> int:
        """Get current queue size."""
        return self.frame_queue.qsize() 