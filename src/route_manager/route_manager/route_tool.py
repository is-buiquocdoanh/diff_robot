import sys
import os
import yaml
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton,
    QFileDialog, QVBoxLayout, QHBoxLayout, QWidget, QListWidget,
    QMessageBox, QShortcut, QScrollArea
)
from PyQt5.QtGui import QPixmap, QPainter, QPen, QKeySequence
from PyQt5.QtCore import Qt, QPoint, QTimer, QSize

class MapWidget(QLabel):
    def __init__(self):
        super().__init__()
        self.setMouseTracking(True)

        self.points = []
        self.current_start = None
        self.current_end = None
        self.pick_mode = False

        self.map_yaml = None
        self.map_resolution = 1.0
        self.map_origin = [0, 0]
        self.original_pixmap = None
        self.zoom_factor = 1.0
        self.scroll_area = None
        self.is_panning = False
        self.last_pan_pos = None

        self.setAlignment(Qt.AlignLeft | Qt.AlignTop)

    def set_scroll_area(self, scroll_area):
        self.scroll_area = scroll_area

    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.map_yaml = yaml.safe_load(f)

        img_path = self.map_yaml['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)

        self.map_resolution = self.map_yaml['resolution']
        self.map_origin = self.map_yaml['origin']

        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            raise ValueError(f"Cannot load map image: {img_path}")

        self.original_pixmap = pixmap
        self.zoom_factor = 1.0
        self.apply_zoom()
        self.update_cursor()

    def set_pick_mode(self, enabled):
        self.pick_mode = enabled
        self.current_start = None
        self.current_end = None
        self.is_panning = False
        self.last_pan_pos = None
        self.update_cursor()
        self.update()

    def update_cursor(self):
        if self.original_pixmap is None:
            self.setCursor(Qt.ArrowCursor)
        elif self.pick_mode:
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.OpenHandCursor)

    def zoom_in(self):
        self.set_zoom(self.zoom_factor * 1.25)

    def zoom_out(self):
        self.set_zoom(self.zoom_factor / 1.25)

    def reset_zoom(self):
        self.set_zoom(1.0)

    def set_zoom(self, zoom_factor, anchor_pos=None):
        if self.original_pixmap is None:
            return

        old_zoom = self.zoom_factor
        self.zoom_factor = max(0.2, min(5.0, zoom_factor))
        if self.zoom_factor == old_zoom:
            return

        self.apply_zoom()

        if anchor_pos is not None and self.scroll_area is not None:
            scale = self.zoom_factor / old_zoom
            hbar = self.scroll_area.horizontalScrollBar()
            vbar = self.scroll_area.verticalScrollBar()
            hbar.setValue(int((hbar.value() + anchor_pos.x()) * scale - anchor_pos.x()))
            vbar.setValue(int((vbar.value() + anchor_pos.y()) * scale - anchor_pos.y()))

    def apply_zoom(self):
        if self.original_pixmap is None:
            return

        size = QSize(
            int(self.original_pixmap.width() * self.zoom_factor),
            int(self.original_pixmap.height() * self.zoom_factor)
        )
        scaled_pixmap = self.original_pixmap.scaled(
            size,
            Qt.KeepAspectRatio,
            Qt.FastTransformation
        )
        self.setPixmap(scaled_pixmap)
        self.adjustSize()
        self.update()

    def is_inside_map(self, p):
        pixmap = self.pixmap()
        if pixmap is None:
            return False

        return 0 <= p.x() < pixmap.width() and 0 <= p.y() < pixmap.height()

    def wheelEvent(self, event):
        if self.original_pixmap is None:
            event.ignore()
            return

        if event.angleDelta().y() > 0:
            zoom_factor = self.zoom_factor * 1.25
        else:
            zoom_factor = self.zoom_factor / 1.25

        self.set_zoom(zoom_factor, event.pos())
        event.accept()

    def mousePressEvent(self, event):
        if (
            self.pick_mode
            and self.pixmap() is not None
            and event.button() == Qt.LeftButton
            and self.is_inside_map(event.pos())
        ):
            self.current_start = event.pos()
            self.current_end = event.pos()
            return

        if (
            self.pixmap() is not None
            and self.is_inside_map(event.pos())
            and (
                event.button() == Qt.MiddleButton
                or (event.button() == Qt.LeftButton and not self.pick_mode)
            )
        ):
            self.is_panning = True
            self.last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.is_panning and self.last_pan_pos is not None:
            self.pan_map(event.pos() - self.last_pan_pos)
            self.last_pan_pos = event.pos()
            event.accept()
            return

        if self.current_start is not None:
            if not self.is_inside_map(event.pos()):
                return
            self.current_end = event.pos()
            self.update()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.is_panning:
            self.is_panning = False
            self.last_pan_pos = None
            self.update_cursor()
            event.accept()
            return

        if self.current_start is None or not self.pick_mode:
            super().mouseReleaseEvent(event)
            return

        end = event.pos()
        if not self.is_inside_map(end):
            self.current_start = None
            self.current_end = None
            self.update()
            return

        # pixel → map
        x1, y1 = self.pixel_to_map(self.current_start)
        x2, y2 = self.pixel_to_map(end)

        yaw = math.atan2(y2 - y1, x2 - x1)

        self.points.append({
            "x": x1,
            "y": y1,
            "yaw": yaw
        })

        self.current_start = None
        self.current_end = None
        self.update()

    def pan_map(self, delta):
        if self.scroll_area is None:
            return

        hbar = self.scroll_area.horizontalScrollBar()
        vbar = self.scroll_area.verticalScrollBar()
        hbar.setValue(hbar.value() - delta.x())
        vbar.setValue(vbar.value() - delta.y())

    def pixel_to_map(self, p):
        px = p.x() / self.zoom_factor
        py = p.y() / self.zoom_factor

        height = self.original_pixmap.height()

        mx = px * self.map_resolution + self.map_origin[0]
        my = (height - py) * self.map_resolution + self.map_origin[1]

        return mx, my

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QPainter(self)
        pen = QPen(Qt.red, 3)
        painter.setPen(pen)

        for p in self.points:
            px, py = self.map_to_pixel(p["x"], p["y"])
            painter.drawEllipse(QPoint(px, py), 5, 5)

            # vẽ hướng
            dx = int(25 * math.cos(p["yaw"]))
            dy = int(-25 * math.sin(p["yaw"]))
            painter.drawLine(QPoint(px, py), QPoint(px + dx, py + dy))

        if self.current_start is not None and self.current_end is not None:
            preview_pen = QPen(Qt.blue, 2)
            painter.setPen(preview_pen)
            painter.drawLine(self.current_start, self.current_end)

    def map_to_pixel(self, x, y):
        height = self.original_pixmap.height()

        px = int((x - self.map_origin[0]) / self.map_resolution * self.zoom_factor)
        py = int(
            (height - (y - self.map_origin[1]) / self.map_resolution)
            * self.zoom_factor
        )

        return px, py


class RouteGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Nav2 Route Tool")

        self.map_widget = MapWidget()
        self.list_widget = QListWidget()

        load_btn = QPushButton("Load Map")
        self.pick_btn = QPushButton("Select Point: OFF")
        self.pick_btn.setCheckable(True)
        save_btn = QPushButton("Save Route")
        delete_btn = QPushButton("Delete Selected Point")
        clear_btn = QPushButton("Clear")
        zoom_in_btn = QPushButton("Zoom In")
        zoom_out_btn = QPushButton("Zoom Out")
        reset_zoom_btn = QPushButton("Reset Zoom")

        load_btn.clicked.connect(self.load_map)
        self.pick_btn.toggled.connect(self.toggle_pick_mode)
        save_btn.clicked.connect(self.save_route)
        delete_btn.clicked.connect(self.delete_selected_point)
        clear_btn.clicked.connect(self.clear_points)
        zoom_in_btn.clicked.connect(self.map_widget.zoom_in)
        zoom_out_btn.clicked.connect(self.map_widget.zoom_out)
        reset_zoom_btn.clicked.connect(self.map_widget.reset_zoom)

        self.delete_shortcut = QShortcut(QKeySequence.Delete, self)
        self.delete_shortcut.activated.connect(self.delete_selected_point)

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.map_widget)
        scroll_area.setWidgetResizable(False)
        self.map_widget.set_scroll_area(scroll_area)

        map_control_layout = QHBoxLayout()
        map_control_layout.addWidget(load_btn)
        map_control_layout.addWidget(self.pick_btn)

        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(zoom_out_btn)
        zoom_layout.addWidget(reset_zoom_btn)
        zoom_layout.addWidget(zoom_in_btn)

        route_control_layout = QHBoxLayout()
        route_control_layout.addWidget(save_btn)
        route_control_layout.addWidget(delete_btn)
        route_control_layout.addWidget(clear_btn)

        control_panel = QWidget()
        control_layout = QVBoxLayout()
        control_layout.addLayout(map_control_layout)
        control_layout.addLayout(zoom_layout)
        control_layout.addLayout(route_control_layout)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(control_panel, 1)
        bottom_layout.addWidget(self.list_widget, 1)

        layout = QVBoxLayout()
        layout.addWidget(scroll_area, 5)
        layout.addLayout(bottom_layout, 1)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

        self.timer_update_list()

    def load_map(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select map yaml")
        if path:
            try:
                self.map_widget.load_map(path)
                self.map_widget.points = []
                self.map_widget.current_start = None
                self.map_widget.current_end = None
                self.list_widget.clear()
            except Exception as exc:
                QMessageBox.critical(self, "Load map failed", str(exc))

    def toggle_pick_mode(self, enabled):
        self.map_widget.set_pick_mode(enabled)
        self.pick_btn.setText("Select Point: ON" if enabled else "Select Point: OFF")

    def save_route(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save route", "route.yaml")
        if not path:
            return

        data = {"route": []}

        for i, p in enumerate(self.map_widget.points):
            data["route"].append({
                "name": f"P{i+1}",
                "x": p["x"],
                "y": p["y"],
                "yaw": p["yaw"]
            })

        with open(path, "w") as f:
            yaml.dump(data, f)

    def clear_points(self):
        self.map_widget.points = []
        self.map_widget.current_start = None
        self.map_widget.current_end = None
        self.map_widget.update()
        self.update_list()

    def delete_selected_point(self):
        row = self.list_widget.currentRow()
        if row < 0:
            QMessageBox.information(self, "Delete point", "Select a point to delete.")
            return

        if row >= len(self.map_widget.points):
            return

        del self.map_widget.points[row]
        self.map_widget.current_start = None
        self.map_widget.current_end = None
        self.map_widget.update()
        self.update_list(selected_row=min(row, len(self.map_widget.points) - 1))

    def update_list(self, selected_row=None):
        if selected_row is None:
            selected_row = self.list_widget.currentRow()

        self.list_widget.clear()
        for i, p in enumerate(self.map_widget.points):
            self.list_widget.addItem(
                f"P{i+1}: x={p['x']:.2f}, y={p['y']:.2f}, yaw={p['yaw']:.2f}"
            )

        if 0 <= selected_row < self.list_widget.count():
            self.list_widget.setCurrentRow(selected_row)

    def timer_update_list(self):
        self.update_list()
        QTimer.singleShot(500, self.timer_update_list)


def main():
    app = QApplication(sys.argv)
    window = RouteGUI()
    window.show()
    sys.exit(app.exec_())
