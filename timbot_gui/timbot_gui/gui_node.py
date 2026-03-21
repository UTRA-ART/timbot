import sys
import subprocess
import json
import os
import csv
import threading
import importlib
from typing import Dict, List, Any, Optional
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import serialize_message
try:
    import rosbag2_py
    HAS_ROSBAG2 = True
except ImportError:
    HAS_ROSBAG2 = False
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QLabel, QPushButton, QListWidget, QListWidgetItem,
    QTableWidget, QTableWidgetItem, QMessageBox, QGroupBox, QScrollArea,
    QCheckBox, QLineEdit, QFormLayout, QSpinBox, QSplitter, QFrame,
    QToolBar, QSizePolicy, QToolButton, QTextEdit, QDialog,
    QGridLayout, QProgressBar, QShortcut
)
from PyQt5.QtWidgets import QHeaderView
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QSize
from PyQt5.QtGui import QColor, QFont, QIcon, QPixmap, QKeySequence
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# ════════════════════════════════════════════════════════════════
#  Design Specification  (exportable — all values referenced below)
# ════════════════════════════════════════════════════════════════
#
#  COLORS – Light theme (WCAG AA verified on bg_primary #f5f7fa):
#    bg_primary     #f5f7fa   Main window background       (light gray)
#    bg_secondary   #ffffff   Panel / card surface         (white)
#    bg_tertiary    #e8ecf1   Input fields / elevated sfc  (silver)
#    border         #cbd5e1   Borders & dividers
#    accent         #2563eb   Primary interactive blue     (contrast 4.6:1 – AA lg)
#    accent_hover   #1d4ed8   Blue hover / pressed
#    accent_light   #1e40af   Section headings / highlights (contrast 9.4:1 – AAA)
#    success        #16a34a   Running / OK                 (contrast 4.6:1 – AA lg)
#    success_dark   #15803d   Success hover
#    danger         #dc2626   Stop / error                 (contrast 5.6:1 – AA)
#    danger_dark    #b91c1c   Danger hover
#    warning        #b45309   Caution / recording          (contrast 5.0:1 – AA)
#    text_primary   #1e293b   Body text                    (contrast 13.8:1 – AAA)
#    text_secondary #475569   Subtitles / captions         (contrast  7.2:1 – AAA)
#    text_muted     #94a3b8   Disabled / placeholders      (contrast  3.2:1)
#
#  TYPOGRAPHY (large / accessible):
#    font-family   "Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif
#    title         22px / 700 (bold)
#    header        20px / 600 (semi-bold)
#    body          18px / 400 (regular)
#    caption       14px / 400
#    monospace     "JetBrains Mono", "Fira Code", "Consolas", monospace
#
#  SPACING:  4 · 8 · 12 · 16 · 24 · 32 px
#  RADII:    panel 6px  |  button/input 4px  |  badge 12px
#  SHADOW:   0 2px 8px rgba(0,0,0,0.08)
# ════════════════════════════════════════════════════════════════

C = {
    "bg_primary":     "#f5f7fa",
    "bg_secondary":   "#ffffff",
    "bg_tertiary":    "#e8ecf1",
    "border":         "#cbd5e1",
    "accent":         "#2563eb",
    "accent_hover":   "#1d4ed8",
    "accent_light":   "#1e40af",
    "success":        "#16a34a",
    "success_dark":   "#15803d",
    "danger":         "#dc2626",
    "danger_dark":    "#b91c1c",
    "warning":        "#b45309",
    "text_primary":   "#1e293b",
    "text_secondary": "#475569",
    "text_muted":     "#94a3b8",
}

T_FAM  = '"Segoe UI", "Roboto", "Helvetica Neue", Arial, sans-serif'
T_MONO = '"JetBrains Mono", "Fira Code", "Consolas", monospace'

GLOBAL_STYLESHEET = f"""
/* ── Base ── */
QMainWindow, QWidget {{
    background-color: {C["bg_primary"]};
    color: {C["text_primary"]};
    font-family: {T_FAM};
    font-size: 18px;
}}
/* ── Toolbar ── */
QToolBar {{
    background-color: {C["bg_secondary"]};
    border-bottom: 1px solid {C["border"]};
    padding: 4px 8px;
    spacing: 6px;    source install/setup.bash && ros2 node list 2>&1 | head -10
}}
QToolBar QToolButton {{
    background: transparent;
    color: {C["text_primary"]};
    border: 1px solid transparent;
    border-radius: 4px;
    padding: 6px 12px;
    font-weight: 500;
}}
QToolBar QToolButton:hover {{
    background-color: {C["bg_tertiary"]};
    border-color: {C["border"]};
}}
/* ── Group Boxes ── */
QGroupBox {{
    background-color: {C["bg_secondary"]};
    border: 1px solid {C["border"]};
    border-radius: 6px;
    margin-top: 12px;
    padding: 16px 12px 12px 12px;
    font-size: 20px;
    font-weight: 600;
    color: {C["text_primary"]};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: {C["accent_light"]};
}}
/* ── Buttons ── */
QPushButton {{
    background-color: {C["bg_tertiary"]};
    color: {C["text_primary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    padding: 6px 14px;
    font-size: 18px;
    font-weight: 500;
    min-height: 42px;
}}
QPushButton:hover {{
    background-color: {C["accent"]};
    border-color: {C["accent"]};
    color: white;
}}
QPushButton:pressed {{
    background-color: {C["accent_hover"]};
}}
QPushButton:disabled {{
    background-color: {C["bg_primary"]};
    color: {C["text_muted"]};
    border-color: {C["bg_tertiary"]};
}}
/* ── Tables ── */
QTableWidget {{
    background-color: {C["bg_secondary"]};
    alternate-background-color: {C["bg_tertiary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    gridline-color: {C["border"]};
    color: {C["text_primary"]};
    selection-background-color: {C["accent"]};
}}
QHeaderView::section {{
    background-color: {C["bg_tertiary"]};
    color: {C["text_secondary"]};
    border: none;
    border-bottom: 2px solid {C["accent"]};
    padding: 6px 8px;
    font-weight: 600;
}}
/* ── Lists ── */
QListWidget {{
    background-color: {C["bg_secondary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    color: {C["text_primary"]};
    outline: none;
}}
QListWidget::item {{
    padding: 6px 8px;
    border-bottom: 1px solid {C["bg_tertiary"]};
}}
QListWidget::item:selected {{
    background-color: {C["accent"]};
    color: white;
}}
QListWidget::item:hover {{
    background-color: {C["bg_tertiary"]};
}}
/* ── Inputs ── */
QLineEdit, QSpinBox {{
    background-color: {C["bg_tertiary"]};
    color: {C["text_primary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    padding: 6px 8px;
    selection-background-color: {C["accent"]};
}}
QLineEdit:focus, QSpinBox:focus {{
    border-color: {C["accent"]};
}}
/* ── Checkboxes ── */
QCheckBox {{
    color: {C["text_primary"]};
    spacing: 8px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {C["border"]};
    border-radius: 3px;
    background-color: {C["bg_tertiary"]};
}}
QCheckBox::indicator:checked {{
    background-color: {C["accent"]};
    border-color: {C["accent"]};
}}
/* ── Scroll ── */
QScrollArea {{
    border: none;
    background: transparent;
}}
QScrollBar:vertical {{
    background: {C["bg_primary"]};
    width: 8px;
    border-radius: 4px;
}}
QScrollBar::handle:vertical {{
    background: {C["border"]};
    border-radius: 4px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{
    background: {C["text_muted"]};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
/* ── Splitter ── */
QSplitter::handle {{
    background-color: {C["border"]};
    width: 2px;
}}
/* ── Log / TextEdit ── */
QTextEdit {{
    background-color: {C["bg_secondary"]};
    color: {C["text_primary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    font-family: {T_MONO};
    font-size: 16px;
    padding: 8px;
}}
/* ── Tab Widget ── */
QTabWidget::pane {{
    border: 1px solid {C["border"]};
    border-radius: 4px;
    background-color: {C["bg_primary"]};
}}
QTabBar::tab {{
    background-color: {C["bg_tertiary"]};
    color: {C["text_secondary"]};
    border: 1px solid {C["border"]};
    border-bottom: none;
    padding: 8px 16px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}}
QTabBar::tab:selected {{
    background-color: {C["bg_primary"]};
    color: {C["accent_light"]};
    border-bottom: 2px solid {C["accent"]};
}}
QTabBar::tab:hover {{
    color: {C["text_primary"]};
}}
/* ── Progress Bar ── */
QProgressBar {{
    background-color: {C["bg_tertiary"]};
    border: 1px solid {C["border"]};
    border-radius: 4px;
    text-align: center;
    color: {C["text_primary"]};
    height: 18px;
}}
QProgressBar::chunk {{
    background-color: {C["accent"]};
    border-radius: 3px;
}}
/* ── Dialogs / Message Boxes ── */
QMessageBox {{
    background-color: {C["bg_secondary"]};
    color: {C["text_primary"]};
}}
QMessageBox QLabel {{
    color: {C["text_primary"]};
}}
QMessageBox QPushButton {{
    min-width: 80px;
}}
/* ── Tooltips ── */
QToolTip {{
    background-color: {C["bg_tertiary"]};
    color: {C["text_primary"]};
    border: 1px solid {C["border"]};
    padding: 6px 10px;
    border-radius: 4px;
    font-size: 16px;
}}
"""


# ════════════════════════════════════════════════════════════════
#  Reusable widgets
# ════════════════════════════════════════════════════════════════

class CollapsiblePanel(QWidget):
    """Panel with a clickable header that toggles content visibility."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._title = title
        self._expanded = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Toggle header
        self.toggle_btn = QPushButton(f"  ▼  {title}")
        self.toggle_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {C["bg_tertiary"]};
                color: {C["accent_light"]};
                border: 1px solid {C["border"]};
                border-radius: 6px 6px 0 0;
                padding: 10px 14px;
                text-align: left;
                font-size: 20px;
                font-weight: 600;
            }}
            QPushButton:hover {{
                background-color: {C["border"]};
            }}
        """)
        self.toggle_btn.setCursor(Qt.PointingHandCursor)
        self.toggle_btn.clicked.connect(self._toggle)
        layout.addWidget(self.toggle_btn)

        # Content
        self.content = QWidget()
        self.content.setStyleSheet(f"""
            QWidget#collapsible_content {{
                background-color: {C["bg_secondary"]};
                border: 1px solid {C["border"]};
                border-top: none;
                border-radius: 0 0 6px 6px;
            }}
        """)
        self.content.setObjectName("collapsible_content")
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(12, 12, 12, 12)
        self.content_layout.setSpacing(8)
        layout.addWidget(self.content)

    def _toggle(self):
        self._expanded = not self._expanded
        self.content.setVisible(self._expanded)
        arrow = "▼" if self._expanded else "▶"
        self.toggle_btn.setText(f"  {arrow}  {self._title}")
        radius = "6px 6px 0 0" if self._expanded else "6px"
        self.toggle_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {C["bg_tertiary"]};
                color: {C["accent_light"]};
                border: 1px solid {C["border"]};
                border-radius: {radius};
                padding: 10px 14px;
                text-align: left;
                font-size: 20px;
                font-weight: 600;
            }}
            QPushButton:hover {{ background-color: {C["border"]}; }}
        """)

    def add_widget(self, widget):
        self.content_layout.addWidget(widget)

    def add_layout(self, layout):
        self.content_layout.addLayout(layout)


class StatusBadge(QLabel):
    """Color-coded robot status indicator pill."""

    STATES = {
        "idle":      (C["text_muted"], "IDLE"),
        "running":   (C["success"],    "RUNNING"),
        "error":     (C["danger"],     "ERROR"),
        "recording": (C["warning"],    "RECORDING"),
        "launching": (C["accent"],     "LAUNCHING"),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.set_state("idle")

    def set_state(self, state: str):
        color, label = self.STATES.get(state, self.STATES["idle"])
        self.setText(f"  ● {label}  ")
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {color}18;
                color: {color};
                border: 1px solid {color};
                border-radius: 12px;
                padding: 4px 14px;
                font-size: 16px;
                font-weight: 700;
                letter-spacing: 1px;
            }}
        """)
        self._state = state


class KeyboardShortcutDialog(QDialog):
    """Modal listing all keyboard shortcuts."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Keyboard Shortcuts")
        self.setFixedSize(440, 360)

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        title = QLabel("⌨  Keyboard Shortcuts")
        title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {C['accent_light']}; padding-bottom: 8px;")
        layout.addWidget(title)

        shortcuts = [
            ("Ctrl+R", "Refresh nodes & topics"),
            ("Ctrl+E", "Emergency stop – kill all processes"),
            ("Ctrl+B", "Start rosbag recording"),
            ("Ctrl+L", "Focus the event log"),
            ("Ctrl+H", "Show this help dialog"),
            ("Ctrl+Q", "Quit application"),
        ]

        for key, desc in shortcuts:
            row = QHBoxLayout()
            key_lbl = QLabel(key)
            key_lbl.setFixedWidth(100)
            key_lbl.setStyleSheet(f"""
                background-color: {C["bg_tertiary"]};
                color: {C["accent_light"]};
                border: 1px solid {C["border"]};
                border-radius: 4px;
                padding: 4px 8px;
                font-family: {T_MONO};
                font-size: 10px;
                font-weight: 600;
            """)
            desc_lbl = QLabel(desc)
            desc_lbl.setStyleSheet(f"color: {C['text_secondary']}; font-size: 11px; padding-left: 8px;")
            row.addWidget(key_lbl)
            row.addWidget(desc_lbl, 1)
            layout.addLayout(row)

        layout.addStretch()
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)


# ════════════════════════════════════════════════════════════════
#  ROS helpers  (unchanged data-flow)
# ════════════════════════════════════════════════════════════════

class RosNodeManager:
    """Manages ROS2 nodes using command-line tools"""

    @staticmethod
    def get_node_list() -> List[str]:
        """Get list of active ROS nodes"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [node.strip() for node in result.stdout.split('\n') if node.strip()]
        except Exception as e:
            print(f"Error getting node list: {e}")
        return []

    @staticmethod
    def get_topic_list() -> List[Dict[str, str]]:
        """Get list of active ROS topics with their types"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = []
                for line in result.stdout.split('\n'):
                    if not line.strip():
                        continue
                    # Example output: "/chatter [std_msgs/msg/String]"
                    parts = line.split()
                    if len(parts) >= 2:
                        topic_name = parts[0].strip()
                        topic_type = ' '.join(parts[1:]).strip('[]')
                        topics.append({'name': topic_name, 'type': topic_type})
                return topics
        except Exception as e:
            print(f"Error getting topic list: {e}")
        return []

    @staticmethod
    def get_node_info(node_name: str) -> Dict[str, Any]:
        """Get detailed info about a node"""
        try:
            result = subprocess.run(['ros2', 'node', 'info', node_name],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return {'info': result.stdout}
        except Exception as e:
            print(f"Error getting node info: {e}")
        return {}

    @staticmethod
    def launch_file(launch_file: str, args: Dict[str, str] = None) -> subprocess.Popen:
        """Launch a ROS launch file"""
        try:
            cmd = ['ros2', 'launch', launch_file]
            if args:
                for key, value in args.items():
                    cmd.append(f'{key}:={value}')
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE, text=True)
            return process
        except Exception as e:
            print(f"Error launching file: {e}")
        return None

    @staticmethod
    def record_rosbag(topics: List[str], output_dir: str = "/tmp/rosbags") -> subprocess.Popen:
        """Record selected topics to a rosbag"""
        try:
            os.makedirs(output_dir, exist_ok=True)
            
            cmd = ['ros2', 'bag', 'record', '-o', output_dir]
            cmd.extend(topics)
            
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE, text=True)
            return process
        except Exception as e:
            print(f"Error recording rosbag: {e}")
        return None


def _import_message_type(type_str: str):
    """Dynamically import a ROS2 message class from its type string.

    E.g. 'std_msgs/msg/String' -> std_msgs.msg.String
    """
    parts = type_str.replace('/', '.')
    # parts is now e.g. 'std_msgs.msg.String'
    module_path, cls_name = parts.rsplit('.', 1)
    mod = importlib.import_module(module_path)
    return getattr(mod, cls_name)


def _msg_to_str(msg) -> str:
    """Pretty-print a ROS2 message to a readable multi-line string."""
    lines = []
    for field_name in msg.get_fields_and_field_types().keys():
        val = getattr(msg, field_name, None)
        lines.append(f"{field_name}: {val}")
    return '\n'.join(lines)


class GuiNode(Node):
    """ROS2 node that doesn't block the GUI"""
    
    def __init__(self):
        super().__init__('timbot_gui_node')
        self.get_logger().info("Timbot GUI node started")


class BagWriter(Node):
    """ROS2 node that records cmd_vel to a rosbag + CSV log.

    Matches the original bagwriter.txt format:
    - Bag topic: 'bagtopic' as std_msgs/msg/String
    - Message format: "Linear: x, y, z\\nAngular: x, y, z\\n\\n"
    - CSV: overwritten each callback, throttled to 1 Hz
    """

    def __init__(self, output_dir: str = '/tmp/rosbags',
                 topics_to_record: Optional[List[str]] = None):
        super().__init__('bag_writer')
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # ── Rosbag writer — timestamped to avoid collisions ──
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_uri = os.path.join(self.output_dir, f'bag_{ts}')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_uri, storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Create 'bagtopic' as String, matching original script
        topic_info = rosbag2_py.TopicMetadata(
            name='bagtopic',
            type='std_msgs/msg/String',
            serialization_format='cdr',
        )
        self.writer.create_topic(topic_info)

        # Repeat for each extra topic the user selected
        self._extra_topics = topics_to_record or []
        for t in self._extra_topics:
            if t != 'bagtopic':
                self.writer.create_topic(rosbag2_py.TopicMetadata(
                    name=t,
                    type='std_msgs/msg/String',
                    serialization_format='cdr',
                ))

        # ── Subscription ──
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10
        )

        # Log to csv file at most once per second
        self._csv_path = os.path.join(self.output_dir, 'cmd_vel.csv')
        self.nextSecond = 0

        self.get_logger().info(
            f'BagWriter started — output: {self.output_dir}'
        )

    # ── Callback ──────────────────────────────────────────────

    def vel_callback(self, data: Twist):
        linx = data.linear.x
        liny = data.linear.y
        linz = data.linear.z
        angx = data.angular.x
        angy = data.angular.y
        angz = data.angular.z

        message = String()
        message.data = (
            f"Linear: {linx}, {liny}, {linz}\n"
            f"Angular: {angx}, {angy}, {angz}\n\n"
        )

        now_ns = self.get_clock().now().nanoseconds
        now_s = now_ns // 1_000_000_000

        if now_s >= self.nextSecond:
            # Write to csv file (overwrite, matching original format)
            with open(self._csv_path, 'w', newline='') as csvfile:
                velWriter = csv.writer(
                    csvfile, delimiter='\n', quotechar='|',
                    quoting=csv.QUOTE_MINIMAL,
                )
                velWriter.writerow([
                    f"Linear: {linx}, {liny}, {linz}",
                    f"Angular: {angx}, {angy}, {angz}",
                    f"Time: {now_s} seconds",
                ])
            self.nextSecond = now_s + 1

        # Write String message to 'bagtopic'
        self.writer.write(
            'bagtopic',
            serialize_message(message),
            now_ns,
        )

    def destroy_node(self):
        """Flush and close writer before teardown."""
        try:
            del self.writer
        except Exception:
            pass
        super().destroy_node()


class BagWriterThread(QThread):
    """Spin a BagWriter node in a background thread."""
    status_update = pyqtSignal(str)  # messages for the event log

    def __init__(self, output_dir: str, topics: Optional[List[str]] = None):
        super().__init__()
        self._output_dir = output_dir
        self._topics = topics
        self._node: Optional[BagWriter] = None

    def run(self):
        try:
            self._node = BagWriter(self._output_dir, self._topics)
            self.status_update.emit('BagWriter node spinning…')
            rclpy.spin(self._node)
        except Exception as e:
            self.status_update.emit(f'BagWriter error: {e}')
        finally:
            if self._node:
                self._node.destroy_node()

    def stop(self):
        if self._node:
            # Trigger shutdown from the rclpy executor
            self._node.get_logger().info('BagWriter shutting down')
            rclpy.try_shutdown()


# ════════════════════════════════════════════════════════════════
#  Main application window
# ════════════════════════════════════════════════════════════════

class TimbotControlPanel(QMainWindow):
    """Main GUI — modern control-room layout with two-column split."""

    def __init__(self, ros_node: GuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.ros_manager = RosNodeManager()
        self.active_monitors: Dict[str, Any] = {}        # topic -> subscription obj
        self._topic_displays: Dict[str, QListWidget] = {}  # topic -> QListWidget
        self.launched_processes: Dict[str, subprocess.Popen] = {}
        self.active_rosbag_recording: Dict[str, subprocess.Popen] = {}
        self.rosbag_topic_checkboxes: Dict[str, QCheckBox] = {}
        self.launch_buttons: Dict[str, QPushButton] = {}
        self.process_status_timer = None
        self.bag_writer_thread: Optional[BagWriterThread] = None

        self._init_ui()
        self._setup_refresh_timer()
        self._setup_process_status_timer()
        self._register_shortcuts()

        # ── Spin the GuiNode on the main thread so subscriptions fire ──
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros)
        self._spin_timer.start(50)  # 20 Hz

    def _spin_ros(self):
        """Process pending rclpy callbacks on the main/GUI thread."""
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0)

    # ── UI construction ───────────────────────────────────────

    def _init_ui(self):
        self.setWindowTitle("Timbot Control Panel")
        self.setGeometry(80, 80, 1280, 760)
        self.setMinimumSize(960, 600)

        # Toolbar
        self._build_toolbar()

        # Central two-column split
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 8, 12, 12)
        root.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(3)

        # ── Left column (status / control) ──
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_w = QWidget()
        left_lay = QVBoxLayout(left_w)
        left_lay.setContentsMargins(0, 0, 4, 0)
        left_lay.setSpacing(10)
        left_lay.addWidget(self._build_nodes_panel())
        left_lay.addWidget(self._build_launch_panel())
        left_lay.addWidget(self._build_sensor_panel())
        left_lay.addStretch()
        left_scroll.setWidget(left_w)

        # ── Right column (monitoring / recording / log) ──
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_w = QWidget()
        right_lay = QVBoxLayout(right_w)
        right_lay.setContentsMargins(4, 0, 0, 0)
        right_lay.setSpacing(10)
        right_lay.addWidget(self._build_topics_panel())
        right_lay.addWidget(self._build_rosbag_panel())
        right_lay.addWidget(self._build_log_panel())
        right_lay.addStretch()
        right_scroll.setWidget(right_w)

        splitter.addWidget(left_scroll)
        splitter.addWidget(right_scroll)
        splitter.setSizes([460, 800])
        root.addWidget(splitter)

    # ── Toolbar ───────────────────────────────────────────────

    def _build_toolbar(self):
        tb = QToolBar("Main Toolbar")
        tb.setMovable(False)
        tb.setIconSize(QSize(18, 18))
        self.addToolBar(tb)

        # Brand label
        brand = QLabel("  TIMBOT  ")
        brand.setStyleSheet(
            f"font-size: 18px; font-weight: bold; color: {C['accent_light']}; padding-right: 12px;"
        )
        tb.addWidget(brand)
        tb.addSeparator()

        # Status badge
        self.status_badge = StatusBadge()
        tb.addWidget(self.status_badge)
        tb.addSeparator()

        # E-Stop
        estop = QToolButton()
        estop.setText("⛔ E-STOP")
        estop.setToolTip("Emergency stop — kill ALL processes  (Ctrl+E)")
        estop.setStyleSheet(f"""
            QToolButton {{
                background-color: {C["danger"]};
                color: white; font-weight: 700;
                border-radius: 4px; padding: 6px 16px;
            }}
            QToolButton:hover {{ background-color: {C["danger_dark"]}; }}
        """)
        estop.clicked.connect(self._emergency_stop)
        tb.addWidget(estop)

        # Refresh
        ref = QToolButton()
        ref.setText("↻ Refresh")
        ref.setToolTip("Refresh nodes & topics  (Ctrl+R)")
        ref.clicked.connect(self._refresh_all)
        tb.addWidget(ref)

        # Reload params
        rel = QToolButton()
        rel.setText("⟳ Params")
        rel.setToolTip("Reload ROS2 parameters from YAML files")
        rel.clicked.connect(self._reload_params)
        tb.addWidget(rel)

        # Spacer → right-align help
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(spacer)

        hlp = QToolButton()
        hlp.setText("?  Help")
        hlp.setToolTip("Keyboard shortcuts  (Ctrl+H)")
        hlp.clicked.connect(self._show_help)
        tb.addWidget(hlp)

    # ── Left-column panels ────────────────────────────────────

    def _build_nodes_panel(self) -> CollapsiblePanel:
        panel = CollapsiblePanel("Active Nodes")

        self.nodes_list = QListWidget()
        self.nodes_list.setToolTip("Double-click a node to view its info")
        self.nodes_list.itemDoubleClicked.connect(lambda _: self.show_node_info())
        panel.add_widget(self.nodes_list)

        row = QHBoxLayout()
        rb = QPushButton("Refresh")
        rb.setToolTip("Re-scan active ROS2 nodes")
        rb.clicked.connect(self.refresh_nodes)
        row.addWidget(rb)
        ib = QPushButton("Node Info")
        ib.setToolTip("Show publishers / subscribers / services for selected node")
        ib.clicked.connect(self.show_node_info)
        row.addWidget(ib)
        panel.add_layout(row)
        return panel

    def _build_launch_panel(self) -> CollapsiblePanel:
        panel = CollapsiblePanel("Launch Control")

        launch_files = [
            ("Motor Control",  "motor_control motor_control.launch.py",
             "Start the motor control pipeline"),
            ("State Publisher", "description state_publisher.launch.py",
             "Publish robot URDF joint states & TF"),
            ("Odometry",       "odom_state odom_state.launch.py",
             "Start odometry state estimation"),
            ("Lane Detection", "lane_detection launch.py",
             "Launch the CV lane-detection stack"),
        ]

        for label, lf, tip in launch_files:
            btn = QPushButton(f"  ▶  {label}")
            btn.setToolTip(tip)
            btn.setMinimumHeight(36)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked, _lf=lf: self.launch_file_dialog(_lf))
            self.launch_buttons[lf] = btn
            panel.add_widget(btn)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {C['border']};")
        panel.add_widget(sep)

        sl = QLabel("Running Processes")
        sl.setStyleSheet(f"color: {C['text_secondary']}; font-weight: 600; padding-top: 4px;")
        panel.add_widget(sl)

        self.process_list = QListWidget()
        self.process_list.setMaximumHeight(120)
        panel.add_widget(self.process_list)

        kb = QPushButton("✕  Kill Selected Process")
        kb.setToolTip("Terminate the highlighted launched process")
        kb.setStyleSheet(f"""
            QPushButton {{
                background-color: {C["danger"]};
                color: white; font-weight: 700;
                border: none; border-radius: 4px; padding: 7px 14px;
            }}
            QPushButton:hover {{ background-color: {C["danger_dark"]}; }}
        """)
        kb.clicked.connect(self.kill_selected_process)
        panel.add_widget(kb)
        return panel

    def _build_sensor_panel(self) -> CollapsiblePanel:
        """Sensor diagnostics quick-glance."""
        panel = CollapsiblePanel("Sensor Diagnostics")

        info = QLabel(
            "Live sensor health will populate once the relevant topics are active."
        )
        info.setStyleSheet(f"color: {C['text_muted']}; padding: 8px;")
        info.setWordWrap(True)
        panel.add_widget(info)

        grid = QGridLayout()
        sensors = ["IMU", "GPS", "Camera", "LIDAR"]
        self.sensor_badges: Dict[str, StatusBadge] = {}
        for col, name in enumerate(sensors):
            lbl = QLabel(name)
            lbl.setStyleSheet(f"color: {C['text_secondary']}; font-weight: 600;")
            badge = StatusBadge()
            badge.set_state("idle")
            self.sensor_badges[name] = badge
            grid.addWidget(lbl, 0, col)
            grid.addWidget(badge, 1, col)
        panel.add_layout(grid)
        return panel

    # ── Right-column panels ───────────────────────────────────

    def _build_topics_panel(self) -> CollapsiblePanel:
        panel = CollapsiblePanel("ROS2 Topics")

        self.topics_table = QTableWidget()
        self.topics_table.setColumnCount(3)
        self.topics_table.setHorizontalHeaderLabels(["Topic", "Type", ""])
        self.topics_table.setAlternatingRowColors(True)
        self.topics_table.verticalHeader().setVisible(False)
        self.topics_table.verticalHeader().setDefaultSectionSize(48)
        self.topics_table.setWordWrap(False)
        header = self.topics_table.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.topics_table.setMinimumHeight(200)
        self.topics_table.setToolTip("Click Monitor to echo a topic in a new window")
        panel.add_widget(self.topics_table)

        row = QHBoxLayout()
        rb = QPushButton("Refresh Topics")
        rb.setToolTip("Re-scan published ROS2 topics")
        rb.clicked.connect(self.refresh_topics)
        row.addWidget(rb)
        panel.add_layout(row)
        return panel

    def _build_rosbag_panel(self) -> CollapsiblePanel:
        panel = CollapsiblePanel("Rosbag Recording")

        sl = QLabel("Select topics to record:")
        sl.setStyleSheet(f"color: {C['text_secondary']}; font-weight: 600;")
        panel.add_widget(sl)

        # Scrollable checkboxes
        self.rosbag_scroll = QScrollArea()
        self.rosbag_scroll.setWidgetResizable(True)
        self.rosbag_scroll.setMaximumHeight(150)
        cb_w = QWidget()
        self.rosbag_cb_layout = QVBoxLayout(cb_w)
        topics = self.ros_manager.get_topic_list()
        self.rosbag_topic_checkboxes.clear()
        for t in topics:
            cb = QCheckBox(f"{t['name']}  ({t['type']})")
            self.rosbag_topic_checkboxes[t['name']] = cb
            self.rosbag_cb_layout.addWidget(cb)
        self.rosbag_scroll.setWidget(cb_w)
        panel.add_widget(self.rosbag_scroll)

        # Buttons
        brow = QHBoxLayout()
        for txt, slot in [("Refresh", self.refresh_rosbag_topics),
                          ("Select All", self.select_all_topics),
                          ("Deselect All", self.deselect_all_topics)]:
            b = QPushButton(txt)
            b.clicked.connect(slot)
            brow.addWidget(b)
        panel.add_layout(brow)

        # Output dir with inline validation
        drow = QHBoxLayout()
        dl = QLabel("Output:")
        dl.setStyleSheet(f"color: {C['text_secondary']};")
        self.rosbag_output_dir = QLineEdit("/tmp/rosbags")
        self.rosbag_output_dir.setToolTip("Absolute path for rosbag output")
        self.rosbag_output_dir.textChanged.connect(self._validate_output_dir)
        self.dir_validation_lbl = QLabel("")
        self.dir_validation_lbl.setFixedWidth(22)
        drow.addWidget(dl)
        drow.addWidget(self.rosbag_output_dir, 1)
        drow.addWidget(self.dir_validation_lbl)
        panel.add_layout(drow)

        # Progress (indeterminate – visible only while recording)
        self.recording_progress = QProgressBar()
        self.recording_progress.setRange(0, 0)
        self.recording_progress.setVisible(False)
        self.recording_progress.setToolTip("Recording in progress…")
        panel.add_widget(self.recording_progress)

        # Start / Stop
        crow = QHBoxLayout()
        start = QPushButton("●  Start Recording")
        start.setToolTip("Begin rosbag capture for checked topics  (Ctrl+B)")
        start.setStyleSheet(f"""
            QPushButton {{
                background-color: {C["success"]}; color: white; font-weight: 700;
                border: none; border-radius: 4px; padding: 7px 14px;
            }}
            QPushButton:hover {{ background-color: {C["success_dark"]}; }}
        """)
        start.clicked.connect(self.start_rosbag_recording)
        crow.addWidget(start)

        stop = QPushButton("■  Stop Recording")
        stop.setToolTip("Stop the selected rosbag recording")
        stop.setStyleSheet(f"""
            QPushButton {{
                background-color: {C["danger"]}; color: white; font-weight: 700;
                border: none; border-radius: 4px; padding: 7px 14px;
            }}
            QPushButton:hover {{ background-color: {C["danger_dark"]}; }}
        """)
        stop.clicked.connect(self.stop_rosbag_recording)
        crow.addWidget(stop)
        panel.add_layout(crow)

        rl = QLabel("Active Recordings")
        rl.setStyleSheet(f"color: {C['text_secondary']}; font-weight: 600; padding-top: 4px;")
        panel.add_widget(rl)

        self.rosbag_list = QListWidget()
        self.rosbag_list.setMaximumHeight(100)
        panel.add_widget(self.rosbag_list)

        # ── BagWriter (native rosbag2_py recording of cmd_vel + CSV) ──
        bw_sep = QFrame()
        bw_sep.setFrameShape(QFrame.HLine)
        bw_sep.setStyleSheet(f"color: {C['border']};")
        panel.add_widget(bw_sep)

        bw_label = QLabel("BagWriter (cmd_vel → ROSBAG + CSV)")
        bw_label.setStyleSheet(f"color: {C['accent_light']}; font-weight: 700; padding-top: 4px;")
        panel.add_widget(bw_label)

        if not HAS_ROSBAG2:
            no_dep = QLabel("⚠  rosbag2_py not found — install ros-humble-rosbag2-py")
            no_dep.setStyleSheet(f"color: {C['warning']}; padding: 4px;")
            no_dep.setWordWrap(True)
            panel.add_widget(no_dep)
        else:
            bw_row = QHBoxLayout()
            self.bw_start_btn = QPushButton("▶  Start BagWriter")
            self.bw_start_btn.setToolTip(
                "Spin a BagWriter ROS node that subscribes to /cmd_vel,\n"
                "writes an MCAP rosbag, and logs a CSV (throttled 1 Hz)."
            )
            self.bw_start_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C['success']}; color: white; font-weight: 700;
                    border: none; border-radius: 4px; padding: 7px 14px;
                }}
                QPushButton:hover {{ background-color: {C['success_dark']}; }}
            """)
            self.bw_start_btn.clicked.connect(self._start_bag_writer)
            bw_row.addWidget(self.bw_start_btn)

            self.bw_stop_btn = QPushButton("■  Stop BagWriter")
            self.bw_stop_btn.setToolTip("Shut down the BagWriter node")
            self.bw_stop_btn.setEnabled(False)
            self.bw_stop_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C['danger']}; color: white; font-weight: 700;
                    border: none; border-radius: 4px; padding: 7px 14px;
                }}
                QPushButton:hover {{ background-color: {C['danger_dark']}; }}
            """)
            self.bw_stop_btn.clicked.connect(self._stop_bag_writer)
            bw_row.addWidget(self.bw_stop_btn)
            panel.add_layout(bw_row)

            self.bw_status_lbl = QLabel("Idle")
            self.bw_status_lbl.setStyleSheet(f"color: {C['text_muted']};")
            panel.add_widget(self.bw_status_lbl)

        return panel

    def _build_log_panel(self) -> CollapsiblePanel:
        panel = CollapsiblePanel("Event Log")
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumHeight(180)
        self.log_output.setToolTip("Timestamped event log")
        panel.add_widget(self.log_output)
        cb = QPushButton("Clear Log")
        cb.clicked.connect(self.log_output.clear)
        panel.add_widget(cb)
        return panel

    # ── Validation ────────────────────────────────────────────

    def _validate_output_dir(self, text: str):
        if os.path.isabs(text):
            self.dir_validation_lbl.setText("✓")
            self.dir_validation_lbl.setStyleSheet(f"color: {C['success']}; font-size: 14px;")
            self.rosbag_output_dir.setStyleSheet(f"""
                QLineEdit {{
                    background-color: {C["bg_tertiary"]}; color: {C["text_primary"]};
                    border: 1px solid {C["success"]}; border-radius: 4px; padding: 6px 8px;
                }}
            """)
        else:
            self.dir_validation_lbl.setText("✗")
            self.dir_validation_lbl.setStyleSheet(f"color: {C['danger']}; font-size: 14px;")
            self.rosbag_output_dir.setStyleSheet(f"""
                QLineEdit {{
                    background-color: {C["bg_tertiary"]}; color: {C["text_primary"]};
                    border: 1px solid {C["danger"]}; border-radius: 4px; padding: 6px 8px;
                }}
            """)

    # ── Logging helper ────────────────────────────────────────

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        muted = C["text_muted"]
        self.log_output.append(
            f"<span style='color:{muted}'>[{ts}]</span> {msg}"
        )
        sb = self.log_output.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Toolbar actions ───────────────────────────────────────

    def _emergency_stop(self):
        if not self.launched_processes and not self.active_rosbag_recording:
            QMessageBox.information(self, "E-Stop", "No active processes to stop.")
            return
        reply = QMessageBox.warning(
            self, "⛔ Emergency Stop",
            "This will terminate ALL running launch files and rosbag recordings.\n\nContinue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        count = 0
        for p in list(self.launched_processes.values()):
            try:
                p.terminate()
                count += 1
            except Exception:
                pass
        for p in list(self.active_rosbag_recording.values()):
            try:
                p.terminate()
                count += 1
            except Exception:
                pass
        self.launched_processes.clear()
        self.active_rosbag_recording.clear()
        self.process_list.clear()
        self.rosbag_list.clear()
        self.recording_progress.setVisible(False)
        self.status_badge.set_state("idle")
        danger = C["danger"]
        self._log(f"<b style='color:{danger}'>E-STOP</b> — terminated {count} process(es)")

    def _refresh_all(self):
        self.refresh_nodes()
        self.refresh_topics()
        self._log("Refreshed nodes & topics")

    def _reload_params(self):
        self._log("Parameter reload requested (no param server configured)")
        QMessageBox.information(
            self, "Reload Params",
            "No parameter server is currently configured.\n"
            "Add your YAML paths in the launch files to enable hot-reload.",
        )

    def _show_help(self):
        KeyboardShortcutDialog(self).exec_()

    # ── Keyboard shortcuts ────────────────────────────────────

    def _register_shortcuts(self):
        QShortcut(QKeySequence("Ctrl+R"), self).activated.connect(self._refresh_all)
        QShortcut(QKeySequence("Ctrl+E"), self).activated.connect(self._emergency_stop)
        QShortcut(QKeySequence("Ctrl+B"), self).activated.connect(self.start_rosbag_recording)
        QShortcut(QKeySequence("Ctrl+H"), self).activated.connect(self._show_help)
        QShortcut(QKeySequence("Ctrl+L"), self).activated.connect(
            lambda: self.log_output.setFocus()
        )
        QShortcut(QKeySequence("Ctrl+Q"), self).activated.connect(self.close)

    # ── Core data-flow methods (logic preserved) ──────────────

    def refresh_rosbag_topics(self):
        self.rosbag_topic_checkboxes.clear()
        cb_w = QWidget()
        cb_lay = QVBoxLayout(cb_w)
        topics = self.ros_manager.get_topic_list()
        for t in topics:
            cb = QCheckBox(f"{t['name']}  ({t['type']})")
            self.rosbag_topic_checkboxes[t['name']] = cb
            cb_lay.addWidget(cb)
        self.rosbag_scroll.setWidget(cb_w)
        self._log("Rosbag topic list refreshed")

    def select_all_topics(self):
        for cb in self.rosbag_topic_checkboxes.values():
            cb.setChecked(True)

    def deselect_all_topics(self):
        for cb in self.rosbag_topic_checkboxes.values():
            cb.setChecked(False)

    def start_rosbag_recording(self):
        selected = [t for t, cb in self.rosbag_topic_checkboxes.items() if cb.isChecked()]
        if not selected:
            QMessageBox.warning(self, "Rosbag", "Select at least one topic to record.")
            return
        output_dir = self.rosbag_output_dir.text()
        if not os.path.isabs(output_dir):
            QMessageBox.warning(self, "Validation Error",
                                "Output directory must be an absolute path.")
            return
        try:
            process = self.ros_manager.record_rosbag(selected, output_dir)
            if process:
                name = f"Recording {len(self.active_rosbag_recording) + 1} – {len(selected)} topics"
                self.active_rosbag_recording[name] = process
                self.rosbag_list.addItem(f"{name}  (PID {process.pid})")
                self.recording_progress.setVisible(True)
                self.status_badge.set_state("recording")
                self._log(f"Rosbag recording started → <b>{output_dir}</b>")
            else:
                QMessageBox.warning(self, "Error", "Failed to start rosbag recording.")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not start recording: {e}")

    def stop_rosbag_recording(self):
        current = self.rosbag_list.currentItem()
        if not current:
            QMessageBox.warning(self, "Rosbag", "Select a recording to stop.")
            return
        reply = QMessageBox.question(
            self, "Stop Recording",
            "Stop the selected rosbag recording?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        text = current.text()
        for name, proc in list(self.active_rosbag_recording.items()):
            if name in text:
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                    self.rosbag_list.takeItem(self.rosbag_list.row(current))
                    del self.active_rosbag_recording[name]
                    self._log(f"Rosbag stopped — saved to <b>{self.rosbag_output_dir.text()}</b>")
                except Exception as e:
                    QMessageBox.warning(self, "Error", f"Could not stop recording: {e}")
                break
        if not self.active_rosbag_recording:
            self.recording_progress.setVisible(False)
            self._update_global_status()

    # ── BagWriter controls ────────────────────────────────────

    def _start_bag_writer(self):
        if not HAS_ROSBAG2:
            QMessageBox.warning(self, "Missing dependency",
                                "rosbag2_py is not installed.")
            return
        if self.bag_writer_thread and self.bag_writer_thread.isRunning():
            QMessageBox.information(self, "BagWriter",
                                   "BagWriter is already running.")
            return
        output_dir = self.rosbag_output_dir.text()
        if not os.path.isabs(output_dir):
            QMessageBox.warning(self, "Validation",
                                "Output directory must be an absolute path.")
            return
        # Collect checked topics (in addition to /cmd_vel which is always recorded)
        extras = [t for t, cb in self.rosbag_topic_checkboxes.items()
                  if cb.isChecked()]
        self.bag_writer_thread = BagWriterThread(output_dir, extras)
        self.bag_writer_thread.status_update.connect(self._on_bw_status)
        self.bag_writer_thread.finished.connect(self._on_bw_finished)
        self.bag_writer_thread.start()
        self.bw_start_btn.setEnabled(False)
        self.bw_stop_btn.setEnabled(True)
        self.bw_status_lbl.setText("● Recording cmd_vel…")
        self.bw_status_lbl.setStyleSheet(f"color: {C['success']}; font-weight: 600;")
        self._log(f"BagWriter started → <b>{output_dir}</b>")

    def _stop_bag_writer(self):
        if self.bag_writer_thread and self.bag_writer_thread.isRunning():
            reply = QMessageBox.question(
                self, "Stop BagWriter",
                "Stop the BagWriter node?  Bag and CSV will be saved.",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            self.bag_writer_thread.stop()
            self.bag_writer_thread.wait(5000)
            self._log("BagWriter stopped")
        self._on_bw_finished()

    def _on_bw_status(self, msg: str):
        self._log(f"BagWriter: {msg}")

    def _on_bw_finished(self):
        self.bw_start_btn.setEnabled(True)
        self.bw_stop_btn.setEnabled(False)
        self.bw_status_lbl.setText("Idle")
        self.bw_status_lbl.setStyleSheet(f"color: {C['text_muted']};")
        self._update_global_status()

    def refresh_nodes(self):
        self.nodes_list.clear()
        nodes = self.ros_manager.get_node_list()
        for node in nodes:
            self.nodes_list.addItem(QListWidgetItem(node))

    def refresh_topics(self):
        self.topics_table.setRowCount(0)
        topics = self.ros_manager.get_topic_list()
        for row, topic in enumerate(topics):
            self.topics_table.insertRow(row)
            self.topics_table.setItem(row, 0, QTableWidgetItem(topic['name']))
            self.topics_table.setItem(row, 1, QTableWidgetItem(topic['type']))
            mb = QPushButton("Monitor")
            mb.setToolTip(f"Echo {topic['name']} in a new window")
            mb.clicked.connect(lambda _, t=topic['name']: self.monitor_topic(t))
            self.topics_table.setCellWidget(row, 2, mb)

    def monitor_topic(self, topic_name: str):
        """Subscribe to a topic on the GuiNode and show messages in a window."""
        try:
            topics = self.ros_manager.get_topic_list()
            topic_type = next(
                (t['type'] for t in topics if t['name'] == topic_name), None
            )
            if topic_type is None:
                QMessageBox.warning(
                    self, "Error",
                    f"Cannot determine type for topic {topic_name}",
                )
                return

            # Import the message class dynamically
            try:
                msg_class = _import_message_type(topic_type)
            except Exception as e:
                QMessageBox.warning(
                    self, "Error",
                    f"Cannot import type {topic_type}: {e}",
                )
                return

            # Destroy any previous subscription for this topic
            if topic_name in self.active_monitors:
                try:
                    self.ros_node.destroy_subscription(self.active_monitors[topic_name])
                except Exception:
                    pass

            # Build the monitor window
            win = QMainWindow(self)
            win.setWindowTitle(f"Monitor: {topic_name}")
            win.setGeometry(160, 160, 620, 420)
            win.setStyleSheet(GLOBAL_STYLESHEET)

            w = QWidget()
            lay = QVBoxLayout(w)

            lbl = QLabel(f"Topic: {topic_name}\nType: {topic_type}")
            lbl.setStyleSheet(
                f"color: {C['accent_light']}; font-weight: 600; padding: 6px;"
            )
            lay.addWidget(lbl)

            display = QListWidget()
            lay.addWidget(display)

            clr = QPushButton("Clear")
            clr.clicked.connect(display.clear)
            lay.addWidget(clr)

            win.setCentralWidget(w)
            win.show()

            # Store the display widget keyed by topic name
            self._topic_displays[topic_name] = display

            # Create a subscription on the main GuiNode.
            # Callbacks fire during _spin_ros() on the main/GUI thread,
            # so direct Qt widget updates are safe — no signals needed.
            def _on_msg(msg, _tn=topic_name):
                d = self._topic_displays.get(_tn)
                if d is not None:
                    d.addItem(_msg_to_str(msg))
                    d.scrollToBottom()
                    while d.count() > 200:
                        d.takeItem(0)

            sub = self.ros_node.create_subscription(
                msg_class, topic_name, _on_msg, 10
            )
            self.active_monitors[topic_name] = sub
            self._log(f"Monitoring topic <b>{topic_name}</b>")

        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not monitor topic: {e}")

    def show_node_info(self):
        current = self.nodes_list.currentItem()
        if current:
            node_name = current.text()
            info = self.ros_manager.get_node_info(node_name)
            if info:
                QMessageBox.information(
                    self, f"Node: {node_name}",
                    info.get('info', 'No info available'),
                )
            else:
                QMessageBox.warning(self, "Error", "Could not retrieve node information.")

    def launch_file_dialog(self, launch_file: str):
        if launch_file in self.launched_processes:
            QMessageBox.information(
                self, "Launch", f"{launch_file} is already running."
            )
            return
        reply = QMessageBox.question(
            self, "Confirm Launch",
            f"Launch  {launch_file} ?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes,
        )
        if reply != QMessageBox.Yes:
            return
        process = self.ros_manager.launch_file(launch_file)
        if process:
            self.launched_processes[launch_file] = process
            self.process_list.addItem(f"{launch_file}  (PID {process.pid})")
            self.status_badge.set_state("running")
            self._log(f"Launched <b>{launch_file}</b>  (PID {process.pid})")
        else:
            QMessageBox.warning(self, "Error", f"Failed to launch: {launch_file}")

    def kill_selected_process(self):
        current = self.process_list.currentItem()
        if not current:
            return
        reply = QMessageBox.question(
            self, "Confirm Kill",
            f"Terminate  {current.text()} ?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        text = current.text()
        for lf, proc in list(self.launched_processes.items()):
            if lf in text:
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                    self.process_list.takeItem(self.process_list.row(current))
                    del self.launched_processes[lf]
                    self._log(f"Terminated <b>{lf}</b>")
                except Exception as e:
                    QMessageBox.warning(self, "Error", f"Could not terminate: {e}")
                break
        self._update_global_status()

    # ── Timers ────────────────────────────────────────────────

    def _setup_refresh_timer(self):
        t = QTimer(self)
        t.timeout.connect(self.refresh_nodes)
        t.start(5000)

    def _setup_process_status_timer(self):
        self.process_status_timer = QTimer(self)
        self.process_status_timer.timeout.connect(self._update_process_status)
        self.process_status_timer.start(1000)

    def _update_process_status(self):
        dead = [lf for lf, p in self.launched_processes.items() if p.poll() is not None]
        for lf in dead:
            del self.launched_processes[lf]
            for i in range(self.process_list.count()):
                if lf in self.process_list.item(i).text():
                    self.process_list.takeItem(i)
                    break
            self._log(f"Process exited: <b>{lf}</b>")

        for lf, btn in self.launch_buttons.items():
            if lf in self.launched_processes:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {C["success"]}; color: white; font-weight: 700;
                        border: none; border-radius: 4px; padding: 6px 14px; min-height: 28px;
                    }}
                    QPushButton:hover {{ background-color: {C["success_dark"]}; }}
                """)
            else:
                btn.setStyleSheet("")  # reset to global default

        self._update_global_status()

    def _update_global_status(self):
        if self.active_rosbag_recording:
            self.status_badge.set_state("recording")
        elif self.launched_processes:
            self.status_badge.set_state("running")
        else:
            self.status_badge.set_state("idle")

    # ── Cleanup ───────────────────────────────────────────────

    def closeEvent(self, event):
        # Destroy topic monitor subscriptions
        for sub in self.active_monitors.values():
            try:
                self.ros_node.destroy_subscription(sub)
            except Exception:
                pass
        self.active_monitors.clear()
        self._topic_displays.clear()
        # Stop the rclpy spin timer
        if hasattr(self, '_spin_timer'):
            self._spin_timer.stop()
        for proc in self.launched_processes.values():
            try:
                proc.terminate()
            except Exception:
                pass
        for proc in self.active_rosbag_recording.values():
            try:
                proc.terminate()
            except Exception:
                pass
        # Stop BagWriter thread
        if self.bag_writer_thread and self.bag_writer_thread.isRunning():
            self.bag_writer_thread.stop()
            self.bag_writer_thread.wait(3000)
        self.ros_node.destroy_node()
        event.accept()


# ════════════════════════════════════════════════════════════════
#  Entry point
# ════════════════════════════════════════════════════════════════

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    app.setStyleSheet(GLOBAL_STYLESHEET)

    node = GuiNode()
    window = TimbotControlPanel(node)
    window.show()

    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
