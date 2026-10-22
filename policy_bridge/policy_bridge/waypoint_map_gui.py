

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Optional

from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGraphicsItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStyle,
    QTabWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from policy_bridge.waypoint_map import (
    RosMapSpec,
    Waypoint,
    ZoneBounds,
    normalize_waypoint_name,
    normalize_zone_name,
    waypoint_sort_key,
)


class WaypointMapView(QGraphicsView):


    map_clicked = pyqtSignal(float, float)
    map_hovered = pyqtSignal(float, float)
    waypoint_selected = pyqtSignal(str)
    zone_selected = pyqtSignal(str)
    zone_drawn = pyqtSignal(float, float, float, float)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setObjectName("waypointMapView")
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self.setMinimumSize(520, 420)

        self._spec: Optional[RosMapSpec] = None
        self._map_item = None
        self._image_width = 0
        self._image_height = 0
        self._waypoints: Dict[str, Waypoint] = {}
        self._zones: Dict[str, ZoneBounds] = {}
        self._selected = ""
        self._selected_zone = ""
        self._editor_mode = "waypoint"
        self._placement_mode = ""
        self._zone_draw_start: Optional[tuple[float, float]] = None
        self._zone_preview_item = None
        self._overlay_items = []
        self._marker_pixels: Dict[str, tuple[float, float]] = {}
        self._auto_fit = True

    @property
    def image_width(self) -> int:
        return self._image_width

    @property
    def image_height(self) -> int:
        return self._image_height

    def set_map(self, spec: RosMapSpec) -> None:
        pixmap = QPixmap(str(spec.image_path))
        if pixmap.isNull():
            raise ValueError(
                f"Map image could not be displayed: {spec.image_path}"
            )
        self._spec = spec
        self._image_width = pixmap.width()
        self._image_height = pixmap.height()
        self._scene.clear()
        self._overlay_items = []
        self._map_item = self._scene.addPixmap(pixmap)
        self._map_item.setZValue(0)
        self._scene.setSceneRect(0, 0, pixmap.width(), pixmap.height())
        self._auto_fit = True
        self._draw_overlays()
        QTimer.singleShot(0, self.fit_map)

    def set_waypoints(
        self, waypoints: Dict[str, Waypoint], selected: str = ""
    ) -> None:
        self._waypoints = dict(waypoints)
        self._selected = selected
        self._draw_overlays()

    def set_zones(
        self, zones: Dict[str, ZoneBounds], selected: str = ""
    ) -> None:
        self._zones = dict(zones)
        self._selected_zone = selected
        self._draw_overlays()

    def set_editor_mode(self, mode: str) -> None:
        self._editor_mode = "zone" if mode == "zone" else "waypoint"

    def select_waypoint(self, name: str) -> None:
        self._selected = name
        self._draw_overlays()

    def select_zone(self, name: str) -> None:
        self._selected_zone = name
        self._draw_overlays()

    def set_placement_enabled(self, enabled: bool) -> None:
        self._set_placement_mode("waypoint" if enabled else "")

    def set_zone_placement_enabled(self, enabled: bool) -> None:
        self._set_placement_mode("zone" if enabled else "")

    def _set_placement_mode(self, mode: str) -> None:
        self._placement_mode = mode
        self._zone_draw_start = None
        self._remove_zone_preview()
        if mode:
            self.setDragMode(QGraphicsView.NoDrag)
            self.viewport().setCursor(Qt.CrossCursor)
        else:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.viewport().unsetCursor()

    def _remove_zone_preview(self) -> None:
        if (
            self._zone_preview_item is not None
            and self._zone_preview_item.scene() is self._scene
        ):
            self._scene.removeItem(self._zone_preview_item)
        self._zone_preview_item = None

    def fit_map(self) -> None:
        if self._map_item is None:
            return
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        self._auto_fit = True

    def _add_cross(
        self,
        pixel_x: float,
        pixel_y: float,
        color: QColor,
        label: str,
        label_offset: tuple[float, float] = (10.0, -20.0),
    ) -> None:
        pen = QPen(color, 2.0)
        pen.setCosmetic(True)
        for x1, y1, x2, y2 in (
            (pixel_x - 9, pixel_y, pixel_x + 9, pixel_y),
            (pixel_x, pixel_y - 9, pixel_x, pixel_y + 9),
        ):
            item = self._scene.addLine(x1, y1, x2, y2, pen)
            item.setZValue(8)
            self._overlay_items.append(item)
        text = QGraphicsSimpleTextItem(label)
        text.setBrush(QBrush(color))
        text.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        text.setPos(
            pixel_x + label_offset[0], pixel_y + label_offset[1]
        )
        text.setZValue(9)
        self._scene.addItem(text)
        self._overlay_items.append(text)

    def _draw_overlays(self) -> None:
        self._remove_zone_preview()
        for item in self._overlay_items:
            if item.scene() is self._scene:
                self._scene.removeItem(item)
        self._overlay_items = []
        self._marker_pixels = {}
        if self._spec is None or self._map_item is None:
            return

        origin_pixel = self._spec.world_to_pixel(
            0.0, 0.0, self._image_height
        )
        if self._pixel_inside(*origin_pixel):
            self._add_cross(
                origin_pixel[0],
                origin_pixel[1],
                QColor("#c83e3e"),
                "Origin (0, 0)",
                (10.0, 7.0),
            )

        image_origin = self._spec.world_to_pixel(
            self._spec.origin_x,
            self._spec.origin_y,
            self._image_height,
        )
        self._add_cross(
            image_origin[0],
            image_origin[1],
            QColor("#6c7685"),
            "Image origin",
        )

        zone_colors = (
            "#2f69bd",
            "#22836a",
            "#b36a24",
            "#8a55a5",
            "#b43d5b",
            "#27758a",
        )
        for index, name in enumerate(sorted(self._zones)):
            bounds = self._zones[name]
            polygon = self._zone_polygon(bounds)
            selected = name == self._selected_zone
            color = QColor(zone_colors[index % len(zone_colors)])
            fill = QColor(color)
            fill.setAlpha(58 if selected else 35)
            pen = QPen(color, 3.0 if selected else 1.8)
            pen.setCosmetic(True)
            item = self._scene.addPolygon(
                polygon,
                pen,
                QBrush(fill),
            )
            item.setZValue(3 if selected else 2)
            item.setToolTip(
                f"Zone {name}: x {bounds.x_min:.2f} to "
                f"{bounds.x_max:.2f}, y {bounds.y_min:.2f} to "
                f"{bounds.y_max:.2f}"
            )
            self._overlay_items.append(item)

            center_x, center_y = bounds.center
            pixel_x, pixel_y = self._spec.world_to_pixel(
                center_x, center_y, self._image_height
            )
            label = QGraphicsSimpleTextItem(f"Zone {name}")
            label.setBrush(QBrush(color.darker(125)))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            label.setPos(pixel_x - 18, pixel_y - 9)
            label.setZValue(5)
            self._scene.addItem(label)
            self._overlay_items.append(label)

        for name in sorted(self._waypoints, key=waypoint_sort_key):
            x, y, _yaw = self._waypoints[name]
            pixel_x, pixel_y = self._spec.world_to_pixel(
                x, y, self._image_height
            )
            if not self._pixel_inside(pixel_x, pixel_y):
                continue
            self._marker_pixels[name] = (pixel_x, pixel_y)
            selected = name == self._selected
            color = QColor("#2f69bd" if selected else "#22836a")
            radius = 7.5 if selected else 6.5
            marker = self._scene.addEllipse(
                -radius,
                -radius,
                radius * 2,
                radius * 2,
                QPen(QColor("#ffffff"), 1.5),
                QBrush(color),
            )
            marker.setPos(pixel_x, pixel_y)
            marker.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            marker.setZValue(10)
            marker.setToolTip(f"{name}: ({x:.2f}, {y:.2f})")
            self._overlay_items.append(marker)

            label = QGraphicsSimpleTextItem(name)
            label.setBrush(QBrush(QColor("#17324d")))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            label.setPos(pixel_x + 9, pixel_y - 18)
            label.setZValue(11)
            self._scene.addItem(label)
            self._overlay_items.append(label)

    def _zone_polygon(self, bounds: ZoneBounds) -> QPolygonF:
        corners = (
            (bounds.x_min, bounds.y_min),
            (bounds.x_max, bounds.y_min),
            (bounds.x_max, bounds.y_max),
            (bounds.x_min, bounds.y_max),
        )
        return QPolygonF(
            [
                QPointF(
                    *self._spec.world_to_pixel(
                        world_x, world_y, self._image_height
                    )
                )
                for world_x, world_y in corners
            ]
        )

    def _pixel_inside(self, pixel_x: float, pixel_y: float) -> bool:
        return (
            0.0 <= pixel_x <= float(self._image_width)
            and 0.0 <= pixel_y <= float(self._image_height)
        )

    def _waypoint_near(self, pixel_x: float, pixel_y: float) -> str:
        scale = max(0.001, self.transform().m11())
        threshold = 16.0 / scale
        nearest = ""
        nearest_distance = threshold
        for name, (marker_x, marker_y) in self._marker_pixels.items():
            distance = math.hypot(marker_x - pixel_x, marker_y - pixel_y)
            if distance <= nearest_distance:
                nearest = name
                nearest_distance = distance
        return nearest

    def _zone_at_world(self, world_x: float, world_y: float) -> str:
        matches = [
            name
            for name, bounds in self._zones.items()
            if bounds.x_min <= world_x <= bounds.x_max
            and bounds.y_min <= world_y <= bounds.y_max
        ]
        if not matches:
            return ""
        return min(
            matches,
            key=lambda name: (
                (self._zones[name].x_max - self._zones[name].x_min)
                * (self._zones[name].y_max - self._zones[name].y_min),
                name,
            ),
        )

    def _show_zone_preview(
        self, start: tuple[float, float], end: tuple[float, float]
    ) -> None:
        self._remove_zone_preview()
        x_min, x_max = sorted((start[0], end[0]))
        y_min, y_max = sorted((start[1], end[1]))
        bounds = ZoneBounds(x_min, y_min, x_max, y_max)
        pen = QPen(QColor("#2f69bd"), 2.0, Qt.DashLine)
        pen.setCosmetic(True)
        fill = QColor("#2f69bd")
        fill.setAlpha(35)
        self._zone_preview_item = self._scene.addPolygon(
            self._zone_polygon(bounds), pen, QBrush(fill)
        )
        self._zone_preview_item.setZValue(20)

    def mousePressEvent(self, event) -> None:
        position = self.mapToScene(event.pos())
        if event.button() == Qt.LeftButton and self._pixel_inside(
            position.x(), position.y()
        ):
            if self._placement_mode == "zone" and self._spec is not None:
                self._zone_draw_start = self._spec.pixel_to_world(
                    position.x(), position.y(), self._image_height
                )
                event.accept()
                return
            if self._placement_mode == "waypoint" and self._spec is not None:
                world_x, world_y = self._spec.pixel_to_world(
                    position.x(), position.y(), self._image_height
                )
                self.map_clicked.emit(world_x, world_y)
                event.accept()
                return
            if self._editor_mode == "zone" and self._spec is not None:
                world_x, world_y = self._spec.pixel_to_world(
                    position.x(), position.y(), self._image_height
                )
                zone = self._zone_at_world(world_x, world_y)
                if zone:
                    self.zone_selected.emit(zone)
                    event.accept()
                    return
            waypoint = self._waypoint_near(position.x(), position.y())
            if waypoint:
                self.waypoint_selected.emit(waypoint)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._spec is not None:
            position = self.mapToScene(event.pos())
            if self._pixel_inside(position.x(), position.y()):
                world_x, world_y = self._spec.pixel_to_world(
                    position.x(), position.y(), self._image_height
                )
                self.map_hovered.emit(world_x, world_y)
                if self._zone_draw_start is not None:
                    self._show_zone_preview(
                        self._zone_draw_start, (world_x, world_y)
                    )
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if (
            event.button() == Qt.LeftButton
            and self._zone_draw_start is not None
            and self._spec is not None
        ):
            position = self.mapToScene(event.pos())
            pixel_x = min(max(position.x(), 0.0), float(self._image_width))
            pixel_y = min(max(position.y(), 0.0), float(self._image_height))
            end = self._spec.pixel_to_world(
                pixel_x, pixel_y, self._image_height
            )
            start = self._zone_draw_start
            self._zone_draw_start = None
            self._remove_zone_preview()
            x_min, x_max = sorted((start[0], end[0]))
            y_min, y_max = sorted((start[1], end[1]))
            if x_max - x_min > 1e-6 and y_max - y_min > 1e-6:
                self.zone_drawn.emit(x_min, y_min, x_max, y_max)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        if self._map_item is None:
            super().wheelEvent(event)
            return
        current = self.transform().m11()
        factor = 1.18 if event.angleDelta().y() > 0 else 1.0 / 1.18
        target = current * factor
        if 0.08 <= target <= 16.0:
            self.scale(factor, factor)
            self._auto_fit = False
        event.accept()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._auto_fit and self._map_item is not None:
            self.fit_map()


class WaypointMapPage(QWidget):


    map_open_requested = pyqtSignal(str)
    map_selected = pyqtSignal(str)
    map_reload_requested = pyqtSignal()
    waypoint_update_requested = pyqtSignal(str, float, float, float)
    waypoint_add_requested = pyqtSignal(str)
    waypoint_remove_requested = pyqtSignal(str)
    zone_update_requested = pyqtSignal(str, float, float, float, float)
    zone_add_requested = pyqtSignal(str)
    zone_remove_requested = pyqtSignal(str)
    save_requested = pyqtSignal()
    revert_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._spec: Optional[RosMapSpec] = None
        self._waypoints: Dict[str, Waypoint] = {}
        self._zones: Dict[str, ZoneBounds] = {}
        self._dirty = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 14)
        layout.setSpacing(9)

        toolbar = QFrame()
        toolbar.setObjectName("surface")
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(12, 9, 10, 9)
        title = QLabel("Map source")
        title.setObjectName("sectionTitle")
        self.map_selector = QComboBox()
        self.map_selector.setObjectName("mapSelector")
        self.map_selector.setMinimumWidth(230)
        self.map_selector.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.map_selector.setMinimumContentsLength(18)
        self.open_button = QToolButton()
        self.open_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogOpenButton)
        )
        self.open_button.setToolTip("Open ROS map YAML")
        self.reload_button = QToolButton()
        self.reload_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.reload_button.setToolTip("Reload saved map annotations")
        self.fit_button = QPushButton("Fit map")
        self.fit_button.setObjectName("secondaryButton")
        toolbar_layout.addWidget(title)
        toolbar_layout.addSpacing(8)
        toolbar_layout.addWidget(self.map_selector, 1)
        toolbar_layout.addWidget(self.open_button)
        toolbar_layout.addWidget(self.reload_button)
        toolbar_layout.addWidget(self.fit_button)
        layout.addWidget(toolbar)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        map_surface = QFrame()
        map_surface.setObjectName("surface")
        map_layout = QVBoxLayout(map_surface)
        map_layout.setContentsMargins(10, 10, 10, 8)
        map_header = QHBoxLayout()
        self.map_title = QLabel("Map preview")
        self.map_title.setObjectName("sectionTitle")
        self.map_meta = QLabel("Resolution - | Origin -")
        self.map_meta.setObjectName("sectionMeta")
        map_header.addWidget(self.map_title)
        map_header.addStretch(1)
        map_header.addWidget(self.map_meta)
        map_layout.addLayout(map_header)
        self.map_view = WaypointMapView()
        map_layout.addWidget(self.map_view, 1)
        self.cursor_position = QLabel("Map coordinate: -")
        self.cursor_position.setObjectName("sectionMeta")
        map_layout.addWidget(self.cursor_position)
        splitter.addWidget(map_surface)

        controls = QFrame()
        controls.setObjectName("surface")
        controls.setMinimumWidth(285)
        controls.setMaximumWidth(350)
        control_layout = QVBoxLayout(controls)
        control_layout.setContentsMargins(12, 11, 12, 12)
        control_layout.setSpacing(8)

        self.editor_tabs = QTabWidget()
        self.editor_tabs.setObjectName("mapEditorTabs")
        waypoint_tab = QWidget()
        waypoint_layout = QVBoxLayout(waypoint_tab)
        waypoint_layout.setContentsMargins(3, 8, 3, 3)
        waypoint_layout.setSpacing(8)

        list_header = QHBoxLayout()
        list_title = QLabel("Waypoints")
        list_title.setObjectName("sectionTitle")
        self.count_label = QLabel("0")
        self.count_label.setObjectName("sectionMeta")
        list_header.addWidget(list_title)
        list_header.addStretch(1)
        list_header.addWidget(self.count_label)
        waypoint_layout.addLayout(list_header)

        self.waypoint_list = QListWidget()
        self.waypoint_list.setObjectName("waypointList")
        self.waypoint_list.setSelectionMode(
            QAbstractItemView.SingleSelection
        )
        self.waypoint_list.setMinimumHeight(165)
        waypoint_layout.addWidget(self.waypoint_list, 1)

        list_buttons = QHBoxLayout()
        self.add_button = QPushButton("Add waypoint")
        self.add_button.setObjectName("secondaryButton")
        self.remove_button = QPushButton("Remove")
        self.remove_button.setObjectName("dangerButton")
        list_buttons.addWidget(self.add_button, 1)
        list_buttons.addWidget(self.remove_button)
        waypoint_layout.addLayout(list_buttons)

        coordinate_label = QLabel("MAP COORDINATES")
        coordinate_label.setObjectName("fieldLabel")
        waypoint_layout.addWidget(coordinate_label)
        coordinate_form = QFormLayout()
        coordinate_form.setSpacing(7)
        self.x_input = self._coordinate_input("m")
        self.y_input = self._coordinate_input("m")
        self.yaw_input = self._coordinate_input("deg")
        self.yaw_input.setRange(-180.0, 180.0)
        self.yaw_input.setDecimals(1)
        self.yaw_input.setSingleStep(5.0)
        coordinate_form.addRow("X", self.x_input)
        coordinate_form.addRow("Y", self.y_input)
        coordinate_form.addRow("Yaw", self.yaw_input)
        waypoint_layout.addLayout(coordinate_form)

        self.apply_button = QPushButton("Apply coordinates")
        self.apply_button.setObjectName("primaryButton")
        self.place_button = QPushButton("Place on map")
        self.place_button.setObjectName("secondaryButton")
        self.place_button.setCheckable(True)
        waypoint_layout.addWidget(self.apply_button)
        waypoint_layout.addWidget(self.place_button)
        self.editor_tabs.addTab(waypoint_tab, "Waypoints")

        zone_tab = QWidget()
        zone_layout = QVBoxLayout(zone_tab)
        zone_layout.setContentsMargins(3, 8, 3, 3)
        zone_layout.setSpacing(8)
        zone_header = QHBoxLayout()
        zone_title = QLabel("Policy zones")
        zone_title.setObjectName("sectionTitle")
        self.zone_count_label = QLabel("0")
        self.zone_count_label.setObjectName("sectionMeta")
        zone_header.addWidget(zone_title)
        zone_header.addStretch(1)
        zone_header.addWidget(self.zone_count_label)
        zone_layout.addLayout(zone_header)

        self.zone_list = QListWidget()
        self.zone_list.setObjectName("zoneList")
        self.zone_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.zone_list.setMinimumHeight(145)
        zone_layout.addWidget(self.zone_list, 1)

        zone_buttons = QHBoxLayout()
        self.zone_add_button = QPushButton("Add zone")
        self.zone_add_button.setObjectName("secondaryButton")
        self.zone_remove_button = QPushButton("Remove")
        self.zone_remove_button.setObjectName("dangerButton")
        zone_buttons.addWidget(self.zone_add_button, 1)
        zone_buttons.addWidget(self.zone_remove_button)
        zone_layout.addLayout(zone_buttons)

        zone_coordinate_label = QLabel("MAP BOUNDS")
        zone_coordinate_label.setObjectName("fieldLabel")
        zone_layout.addWidget(zone_coordinate_label)
        zone_form = QFormLayout()
        zone_form.setSpacing(7)
        self.zone_x_min_input = self._coordinate_input("m")
        self.zone_y_min_input = self._coordinate_input("m")
        self.zone_x_max_input = self._coordinate_input("m")
        self.zone_y_max_input = self._coordinate_input("m")
        zone_form.addRow("X min", self.zone_x_min_input)
        zone_form.addRow("Y min", self.zone_y_min_input)
        zone_form.addRow("X max", self.zone_x_max_input)
        zone_form.addRow("Y max", self.zone_y_max_input)
        zone_layout.addLayout(zone_form)

        self.zone_apply_button = QPushButton("Apply bounds")
        self.zone_apply_button.setObjectName("primaryButton")
        self.zone_draw_button = QPushButton("Draw on map")
        self.zone_draw_button.setObjectName("secondaryButton")
        self.zone_draw_button.setCheckable(True)
        zone_layout.addWidget(self.zone_apply_button)
        zone_layout.addWidget(self.zone_draw_button)
        self.editor_tabs.addTab(zone_tab, "Zones")
        control_layout.addWidget(self.editor_tabs, 1)

        self.status = QLabel("No map loaded")
        self.status.setObjectName("mapStatusNeutral")
        self.status.setWordWrap(True)
        control_layout.addWidget(self.status)

        persistence = QHBoxLayout()
        self.revert_button = QPushButton("Revert")
        self.revert_button.setObjectName("secondaryButton")
        self.save_button = QPushButton("Save map setup")
        self.save_button.setObjectName("primaryButton")
        persistence.addWidget(self.revert_button)
        persistence.addWidget(self.save_button, 1)
        control_layout.addLayout(persistence)
        splitter.addWidget(controls)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([920, 310])
        layout.addWidget(splitter, 1)

        self.open_button.clicked.connect(self._choose_map)
        self.map_selector.currentIndexChanged.connect(
            self._map_selection_changed
        )
        self.reload_button.clicked.connect(self.map_reload_requested.emit)
        self.fit_button.clicked.connect(self.map_view.fit_map)
        self.waypoint_list.currentTextChanged.connect(
            self._selection_changed
        )
        self.map_view.waypoint_selected.connect(self.select_waypoint)
        self.map_view.zone_selected.connect(self.select_zone)
        self.map_view.map_clicked.connect(self._map_clicked)
        self.map_view.zone_drawn.connect(self._zone_drawn)
        self.map_view.map_hovered.connect(self._map_hovered)
        self.add_button.clicked.connect(self._request_add)
        self.remove_button.clicked.connect(self._request_remove)
        self.apply_button.clicked.connect(self._apply_coordinates)
        self.place_button.toggled.connect(
            self.map_view.set_placement_enabled
        )
        self.zone_list.currentTextChanged.connect(
            self._zone_selection_changed
        )
        self.zone_add_button.clicked.connect(self._request_zone_add)
        self.zone_remove_button.clicked.connect(self._request_zone_remove)
        self.zone_apply_button.clicked.connect(self._apply_zone_bounds)
        self.zone_draw_button.toggled.connect(
            self.map_view.set_zone_placement_enabled
        )
        self.editor_tabs.currentChanged.connect(self._editor_changed)
        self.save_button.clicked.connect(self.save_requested.emit)
        self.revert_button.clicked.connect(self.revert_requested.emit)
        self._set_controls_enabled(False)

    @staticmethod
    def _coordinate_input(suffix: str) -> QDoubleSpinBox:
        field = QDoubleSpinBox()
        field.setRange(-10000.0, 10000.0)
        field.setDecimals(3)
        field.setSingleStep(0.05)
        field.setSuffix(f" {suffix}")
        field.setKeyboardTracking(False)
        return field

    @property
    def dirty(self) -> bool:
        return self._dirty

    @property
    def selected_waypoint(self) -> str:
        item = self.waypoint_list.currentItem()
        return item.text() if item is not None else ""

    @property
    def selected_zone(self) -> str:
        item = self.zone_list.currentItem()
        return item.text() if item is not None else ""

    def set_map(self, spec: RosMapSpec) -> None:
        self._spec = spec
        self.map_view.set_map(spec)
        self.map_title.setText(spec.yaml_path.stem)
        index = self.map_selector.findData(str(spec.yaml_path))
        if index >= 0:
            self.map_selector.blockSignals(True)
            self.map_selector.setCurrentIndex(index)
            self.map_selector.blockSignals(False)
        self.map_selector.setToolTip(str(spec.yaml_path))
        self.map_meta.setText(
            f"{spec.resolution:.3f} m/px | "
            f"Image origin ({spec.origin_x:.2f}, {spec.origin_y:.2f}, "
            f"{math.degrees(spec.origin_yaw):.1f} deg)"
        )
        minimum_x, maximum_x, minimum_y, maximum_y = spec.world_bounds(
            self.map_view.image_width, self.map_view.image_height
        )
        self.x_input.setRange(minimum_x, maximum_x)
        self.y_input.setRange(minimum_y, maximum_y)
        self.x_input.setSingleStep(spec.resolution)
        self.y_input.setSingleStep(spec.resolution)
        for field in (self.zone_x_min_input, self.zone_x_max_input):
            field.setRange(minimum_x, maximum_x)
            field.setSingleStep(spec.resolution)
        for field in (self.zone_y_min_input, self.zone_y_max_input):
            field.setRange(minimum_y, maximum_y)
            field.setSingleStep(spec.resolution)
        self._set_controls_enabled(True)

    def set_map_choices(
        self, paths: list[str], current_path: str = ""
    ) -> None:

        self.map_selector.blockSignals(True)
        self.map_selector.clear()
        current_index = -1
        for path in paths:
            resolved = str(Path(path).expanduser().resolve())
            self.map_selector.addItem(Path(resolved).stem, resolved)
            self.map_selector.setItemData(
                self.map_selector.count() - 1, resolved, Qt.ToolTipRole
            )
            if resolved == current_path:
                current_index = self.map_selector.count() - 1
        if current_index >= 0:
            self.map_selector.setCurrentIndex(current_index)
            self.map_selector.setToolTip(current_path)
        self.map_selector.blockSignals(False)

    def set_waypoints(
        self,
        waypoints: Dict[str, Waypoint],
        selected: str = "",
        dirty: Optional[bool] = None,
    ) -> None:
        previous = selected or self.selected_waypoint
        self._waypoints = dict(waypoints)
        self.waypoint_list.blockSignals(True)
        self.waypoint_list.clear()
        for name in sorted(self._waypoints, key=waypoint_sort_key):
            self.waypoint_list.addItem(name)
        self.waypoint_list.blockSignals(False)
        self.count_label.setText(str(len(self._waypoints)))
        target = previous if previous in self._waypoints else ""
        if not target and self._waypoints:
            target = sorted(self._waypoints, key=waypoint_sort_key)[0]
        if target:
            self.select_waypoint(target)
        else:
            self.map_view.set_waypoints(self._waypoints)
            self.remove_button.setEnabled(False)
            self.apply_button.setEnabled(False)
            self.place_button.setEnabled(False)
        if dirty is not None:
            self.set_dirty(dirty)

    def select_waypoint(self, name: str) -> None:
        if name not in self._waypoints:
            return
        matches = self.waypoint_list.findItems(name, Qt.MatchExactly)
        if matches and self.waypoint_list.currentItem() is not matches[0]:
            self.waypoint_list.setCurrentItem(matches[0])
            return
        self._selection_changed(name)

    def set_zones(
        self,
        zones: Dict[str, ZoneBounds],
        selected: str = "",
        dirty: Optional[bool] = None,
    ) -> None:
        previous = selected or self.selected_zone
        self._zones = dict(zones)
        self.zone_list.blockSignals(True)
        self.zone_list.clear()
        for name in sorted(self._zones):
            self.zone_list.addItem(name)
        self.zone_list.blockSignals(False)
        self.zone_count_label.setText(str(len(self._zones)))
        target = previous if previous in self._zones else ""
        if not target and self._zones:
            target = sorted(self._zones)[0]
        if target:
            self.select_zone(target)
        else:
            self.map_view.set_zones(self._zones)
            self.zone_remove_button.setEnabled(False)
            self.zone_apply_button.setEnabled(False)
            self.zone_draw_button.setEnabled(False)
        if dirty is not None:
            self.set_dirty(dirty)

    def select_zone(self, name: str) -> None:
        if name not in self._zones:
            return
        matches = self.zone_list.findItems(name, Qt.MatchExactly)
        if matches and self.zone_list.currentItem() is not matches[0]:
            self.zone_list.setCurrentItem(matches[0])
            return
        self._zone_selection_changed(name)

    def begin_placement(self, name: str) -> None:
        self.select_waypoint(name)
        self.place_button.setChecked(True)

    def begin_zone_placement(self, name: str) -> None:
        self.editor_tabs.setCurrentIndex(1)
        self.select_zone(name)
        self.zone_draw_button.setChecked(True)

    def set_dirty(self, dirty: bool) -> None:
        self._dirty = bool(dirty)
        self.save_button.setEnabled(self._spec is not None and self._dirty)
        self.revert_button.setEnabled(self._spec is not None and self._dirty)
        if self._dirty:
            self.set_status("Unsaved map setup changes", "busy")

    def set_status(self, text: str, mode: str = "neutral") -> None:
        names = {
            "neutral": "mapStatusNeutral",
            "online": "mapStatusOnline",
            "busy": "mapStatusBusy",
            "error": "mapStatusError",
        }
        self.status.setObjectName(names.get(mode, names["neutral"]))
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.status.setText(text)

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in (
            self.reload_button,
            self.fit_button,
            self.waypoint_list,
            self.editor_tabs,
            self.add_button,
            self.remove_button,
            self.x_input,
            self.y_input,
            self.yaw_input,
            self.apply_button,
            self.place_button,
            self.zone_list,
            self.zone_add_button,
            self.zone_remove_button,
            self.zone_x_min_input,
            self.zone_y_min_input,
            self.zone_x_max_input,
            self.zone_y_max_input,
            self.zone_apply_button,
            self.zone_draw_button,
            self.save_button,
            self.revert_button,
        ):
            widget.setEnabled(enabled)
        self.save_button.setEnabled(enabled and self._dirty)
        self.revert_button.setEnabled(enabled and self._dirty)

    def _choose_map(self) -> None:
        start = (
            str(self._spec.yaml_path.parent)
            if self._spec is not None
            else str(Path.home() / "map")
        )
        path, _selected = QFileDialog.getOpenFileName(
            self,
            "Open ROS map",
            start,
            "ROS map YAML (*.yaml *.yml)",
        )
        if path:
            self.map_open_requested.emit(path)

    def _map_selection_changed(self, index: int) -> None:
        path = str(self.map_selector.itemData(index) or "")
        if not path:
            return
        self.map_selector.setToolTip(path)
        if self._spec is not None and path == str(self._spec.yaml_path):
            return
        self.map_selected.emit(path)

    def _selection_changed(self, name: str) -> None:
        if name not in self._waypoints:
            return
        x, y, yaw = self._waypoints[name]
        self.x_input.setValue(x)
        self.y_input.setValue(y)
        self.yaw_input.setValue(math.degrees(yaw))
        self.remove_button.setEnabled(True)
        self.apply_button.setEnabled(True)
        self.place_button.setEnabled(True)
        self.map_view.set_waypoints(self._waypoints, name)

    def _zone_selection_changed(self, name: str) -> None:
        if name not in self._zones:
            return
        bounds = self._zones[name]
        self.zone_x_min_input.setValue(bounds.x_min)
        self.zone_y_min_input.setValue(bounds.y_min)
        self.zone_x_max_input.setValue(bounds.x_max)
        self.zone_y_max_input.setValue(bounds.y_max)
        self.zone_remove_button.setEnabled(True)
        self.zone_apply_button.setEnabled(True)
        self.zone_draw_button.setEnabled(True)
        self.map_view.set_zones(self._zones, name)

    def _editor_changed(self, index: int) -> None:
        self.place_button.setChecked(False)
        self.zone_draw_button.setChecked(False)
        mode = "zone" if index == 1 else "waypoint"
        self.map_view.set_editor_mode(mode)

    def _map_clicked(self, x: float, y: float) -> None:
        name = self.selected_waypoint
        if not name:
            return
        yaw = math.radians(self.yaw_input.value())
        self.place_button.setChecked(False)
        self.waypoint_update_requested.emit(name, x, y, yaw)

    def _map_hovered(self, x: float, y: float) -> None:
        self.cursor_position.setText(f"Map coordinate: ({x:.2f}, {y:.2f})")

    def _apply_coordinates(self) -> None:
        name = self.selected_waypoint
        if not name:
            return
        self.waypoint_update_requested.emit(
            name,
            self.x_input.value(),
            self.y_input.value(),
            math.radians(self.yaw_input.value()),
        )

    def _zone_drawn(
        self, x_min: float, y_min: float, x_max: float, y_max: float
    ) -> None:
        name = self.selected_zone
        if not name:
            return
        self.zone_draw_button.setChecked(False)
        self.zone_update_requested.emit(
            name, x_min, y_min, x_max, y_max
        )

    def _apply_zone_bounds(self) -> None:
        name = self.selected_zone
        if not name:
            return
        self.zone_update_requested.emit(
            name,
            self.zone_x_min_input.value(),
            self.zone_y_min_input.value(),
            self.zone_x_max_input.value(),
            self.zone_y_max_input.value(),
        )

    def _request_add(self) -> None:
        used = {
            int(name[2:])
            for name in self._waypoints
            if name.startswith("WP") and name[2:].isdigit()
        }
        suggestion = 1
        while suggestion in used:
            suggestion += 1
        value, accepted = QInputDialog.getText(
            self,
            "Add waypoint",
            "Waypoint name",
            text=f"WP{suggestion}",
        )
        if not accepted:
            return
        try:
            name = normalize_waypoint_name(value)
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid waypoint name", str(exc))
            return
        if name in self._waypoints:
            QMessageBox.information(
                self, "Waypoint already exists", f"{name} already exists."
            )
            return
        self.waypoint_add_requested.emit(name)

    def _request_remove(self) -> None:
        name = self.selected_waypoint
        if not name:
            return
        answer = QMessageBox.question(
            self,
            "Remove waypoint",
            f"Remove {name} from this map?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.waypoint_remove_requested.emit(name)

    def _request_zone_add(self) -> None:
        suggestion = next(
            (
                letter
                for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                if letter not in self._zones
            ),
            "",
        )
        if not suggestion:
            QMessageBox.information(
                self,
                "Zone limit reached",
                "All single-letter zone labels are already in use.",
            )
            return
        value, accepted = QInputDialog.getText(
            self,
            "Add policy zone",
            "Zone label (A-Z)",
            text=suggestion,
        )
        if not accepted:
            return
        try:
            name = normalize_zone_name(value)
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid zone label", str(exc))
            return
        if name in self._zones:
            QMessageBox.information(
                self, "Zone already exists", f"Zone {name} already exists."
            )
            return
        self.zone_add_requested.emit(name)

    def _request_zone_remove(self) -> None:
        name = self.selected_zone
        if not name:
            return
        answer = QMessageBox.question(
            self,
            "Remove policy zone",
            f"Remove Zone {name} from this map?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.zone_remove_requested.emit(name)
