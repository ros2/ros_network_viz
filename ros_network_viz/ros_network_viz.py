# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import signal
import sys
import time

import networkx

from PyQt5 import QtCore, QtGui, QtWidgets

from ros_network_viz.ros_graph import ROSGraph, service_is_hidden, topic_is_hidden

# Most of the colors in this palette are from
# https://coolors.co/001219-005f73-0a9396-94d2bd-e9d8a6-ee9b00-ca6702-bb3e03-ae2012-9b2226
COLOR_PALETTE = {
    'connection_line': 'f48c06',
    'connection_line_highlight': 'ffba08',
    'lifecycle_node': '0a9396',
    'component_node': '9b2226',
    'regular_node': '505050',
    'node_border': '323232',
    'node_border_highlight': 'ae2012',
    'node_text': 'e6e6e6',
    'topic': '005f73',
    'service': 'bb3e03',
    'action': 'ca6702',
    'background': '282828',
    'grid_line': '323232',
}


def convert_hex_to_color(data):
    r = None
    g = None
    b = None

    if len(data) == 6:
        r = data[0:2]
        g = data[2:4]
        b = data[4:6]
    elif len(data) == 7 and data[0] == '#':
        r = data[1:3]
        g = data[3:5]
        b = data[5:7]

    if r is not None and g is not None and b is not None:
        return QtGui.QColor(int(r, 16), int(g, 16), int(b, 16))

    raise Exception('Invalid color, must be a hex listing with the # or not on the front')


class ConnectionLine(QtWidgets.QGraphicsPathItem):

    def __init__(self, qos_profile):
        super().__init__()

        self.setZValue(-1)
        self.setAcceptHoverEvents(True)

        if qos_profile is not None:
            text = '\n'.join([
                f'Quality of Service:',
                f'  Reliability: {qos_profile.reliability.name}',
                f'  Durability: {qos_profile.durability.name}',
                f'  Lifespan: {qos_profile.lifespan}',
                f'  Deadline: {qos_profile.deadline}',
                f'  Liveliness: {qos_profile.liveliness.name}',
                f'  Liveliness lease duration: {qos_profile.liveliness_lease_duration}',
            ])
            self.setToolTip(text)

        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['connection_line']), 3)

        self._pen_sel = QtGui.QPen(
            convert_hex_to_color(COLOR_PALETTE['connection_line_highlight']), 5)

        self.setPen(self._pen)

    # PyQt method override
    def hoverEnterEvent(self, event):
        self.setPen(self._pen_sel)
        super().hoverEnterEvent(event)

    # PyQt method override
    def hoverLeaveEvent(self, event):
        self.setPen(self._pen)
        super().hoverLeaveEvent(event)

    def update_path(self, source_point, target_point):
        path = QtGui.QPainterPath()

        path.moveTo(source_point)
        dx = (target_point.x() - source_point.x()) * 0.5
        dy = target_point.y() - source_point.y()
        ctrl1 = QtCore.QPointF(source_point.x() + dx, source_point.y() + dy * 0)
        ctrl2 = QtCore.QPointF(source_point.x() + dx, source_point.y() + dy * 1)
        path.cubicTo(ctrl1, ctrl2, target_point)

        self.setPath(path)


class NodeBox(QtWidgets.QGraphicsObject):

    def __init__(self, name, is_lifecycle, is_component_manager):
        super().__init__()

        self._name = name
        self._is_lifecycle = is_lifecycle
        self._is_component_manager = is_component_manager
        self._lifecycle_state = None
        self._params = {}
        self._managed_nodes = []

        self._radius = 10
        self._base_height = 25
        self._node_border = 2

        self.setZValue(1)

        self.setFlag(QtWidgets.QGraphicsObject.ItemIsMovable)
        self.setFlag(QtWidgets.QGraphicsObject.ItemIsSelectable)

        self._brush = QtGui.QBrush()
        self._brush.setStyle(QtCore.Qt.SolidPattern)
        # TODO(clalancette): What happens if a node is both lifecycle *and* component manager?
        if self._is_lifecycle:
            self._brush.setColor(convert_hex_to_color(COLOR_PALETTE['lifecycle_node']))
        elif self._is_component_manager:
            self._brush.setColor(convert_hex_to_color(COLOR_PALETTE['component_node']))
        else:
            self._brush.setColor(convert_hex_to_color(COLOR_PALETTE['regular_node']))

        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_border']),
                               self._node_border,
                               QtCore.Qt.SolidLine)

        self._pen_sel = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_border_highlight']),
                                   self._node_border,
                                   QtCore.Qt.SolidLine)

        self._text_pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_text']),
                                    0,
                                    QtCore.Qt.SolidLine)

        self._node_text_font = QtGui.QFont('Arial', 12, QtGui.QFont.Bold)

        metrics = QtGui.QFontMetrics(self._node_text_font)
        text_width = metrics.boundingRect(self._name).width() + 14
        text_height = metrics.boundingRect(self._name).height() + 14
        height_margin = int((text_height - self._base_height) * 0.5)
        self._base_width = text_width
        self._text_rect = QtCore.QRect(0, -height_margin, text_width, text_height)

        self.update_tooltip()

        self._anim = QtCore.QPropertyAnimation(self, b'pos')

    # PyQt method override
    def boundingRect(self):
        return QtCore.QRectF(QtCore.QRect(0, 0, self._base_width, self._base_height))

    # PyQt method override
    def mouseMoveEvent(self, event):
        self.scene().update_connections()
        super().mouseMoveEvent(event)

    # PyQt method override
    def paint(self, painter, option, widget):
        # First draw the rounded rectangle the represents the node
        painter.setBrush(self._brush)
        if self.isSelected():
            painter.setPen(self._pen_sel)
        else:
            painter.setPen(self._pen)

        painter.drawRoundedRect(0, 0,
                                self._base_width,
                                self._base_height,
                                self._radius,
                                self._radius)

        # Now draw the node name above the rectangle
        painter.setPen(self._text_pen)
        painter.setFont(self._node_text_font)

        painter.drawText(self._text_rect,
                         QtCore.Qt.AlignCenter,
                         self._name)

    def update_tooltip(self):
        text = ''

        if self._lifecycle_state is not None:
            text += 'Lifecycle state: ' + self._lifecycle_state

        if self._managed_nodes:
            text += 'Managed Nodes:\n'
            for n in self._managed_nodes:
                text += '  ' + n + '\n'
            text = text[:-1]

        if self._params:
            if text:
                text += '\n'
            text += 'Parameters:\n'
            for k, v in self._params.items():
                text += '  ' + k + ' -> ' + str(v) + '\n'
            text = text[:-1]

        if not text:
            text = 'No Parameters'

        self.setToolTip(text)

    def update_params(self, new_params):
        self._params = new_params
        self.update_tooltip()

    def update_lifecycle_state(self, new_state):
        self._lifecycle_state = new_state
        self.update_tooltip()

    def update_managed_nodes(self, new_managed_nodes):
        self._managed_nodes = new_managed_nodes
        self.update_tooltip()

    def animation_finished(self):
        scene = self.scene()
        # the scene can be None if the Node was removed before the animation finished
        if scene:
            scene.complete_animation(self._name)

    def set_position(self, x, y):
        self._anim.setEndValue(QtCore.QPointF(x, y))
        self._anim.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._anim.setDuration(1000)
        self._anim.finished.connect(self.animation_finished)
        self._anim.start()

    def left(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(0, rect.y() + rect.height() * 0.5))

    def right(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(rect.x() + rect.width(),
                                              rect.y() + rect.height() * 0.5))


class ConnectionBox(QtWidgets.QGraphicsObject):

    def __init__(self, name, conn_type, bg_color):
        super().__init__()

        self._name = name

        self._radius = 10
        self._base_height = 25
        self._node_border = 2

        self.setZValue(1)

        self.setToolTip('Type: ' + conn_type)

        self.setFlag(QtWidgets.QGraphicsObject.ItemIsMovable)
        self.setFlag(QtWidgets.QGraphicsObject.ItemIsSelectable)

        self._brush = QtGui.QBrush(convert_hex_to_color(COLOR_PALETTE[bg_color]),
                                   QtCore.Qt.SolidPattern)

        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_border']),
                               self._node_border,
                               QtCore.Qt.SolidLine)

        self._pen_sel = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_border_highlight']),
                                   self._node_border,
                                   QtCore.Qt.SolidLine)

        self._text_pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['node_text']),
                                    0,
                                    QtCore.Qt.SolidLine)

        self._node_text_font = QtGui.QFont('Arial', 10, QtGui.QFont.Normal)

        metrics = QtGui.QFontMetrics(self._node_text_font)
        text_width = metrics.boundingRect(self._name).width() + 14
        text_height = metrics.boundingRect(self._name).height() + 14
        height_margin = int((text_height - self._base_height) * 0.5)
        self._base_width = text_width
        self._text_rect = QtCore.QRect(0, -height_margin, text_width, text_height)

        self._anim = QtCore.QPropertyAnimation(self, b'pos')

    # PyQt method override
    def boundingRect(self):
        return QtCore.QRectF(QtCore.QRect(0, 0, self._base_width, self._base_height))

    # PyQt method override
    def mouseMoveEvent(self, event):
        self.scene().update_connections()
        super().mouseMoveEvent(event)

    # PyQt method override
    def paint(self, painter, option, widget):
        # First draw the rounded rectangle the represents the node
        painter.setBrush(self._brush)
        if self.isSelected():
            painter.setPen(self._pen_sel)
        else:
            painter.setPen(self._pen)

        painter.drawRect(0, 0, self._base_width, self._base_height)

        # Now draw the node name (and lifecycle state, if applicable) above the
        # rectangle
        painter.setPen(self._text_pen)
        painter.setFont(self._node_text_font)

        painter.drawText(self._text_rect,
                         QtCore.Qt.AlignCenter,
                         self._name)

    def animation_finished(self):
        scene = self.scene()
        # the scene can be None if the Node was removed before the animation finished
        if scene:
            scene.complete_animation(self._name)

    def set_position(self, x, y):
        self._anim.setEndValue(QtCore.QPointF(x, y))
        self._anim.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._anim.setDuration(1000)
        self._anim.finished.connect(self.animation_finished)
        self._anim.start()

    def left(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(0, rect.y() + rect.height() * 0.5))

    def right(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(rect.x() + rect.width(),
                                              rect.y() + rect.height() * 0.5))


class NetworkScene(QtWidgets.QGraphicsScene):

    new_node_params_signal = QtCore.pyqtSignal(str, dict, name='newNodeParams')
    new_lifecycle_state_signal = QtCore.pyqtSignal(str, str, name='newLifecycleState')
    new_component_nodes_signal = QtCore.pyqtSignal(str, list, name='newComponentManagerNodes')

    new_nodes_signal = QtCore.pyqtSignal(list, name='newNodes')

    def create_bool_right_click_action(self, name, boolean_variable, callback):
        action = QtWidgets.QAction(name, self)
        action.setCheckable(True)
        action.setChecked(boolean_variable)
        action.triggered.connect(callback)
        return action

    def __init__(self, parent):
        super().__init__(parent)

        self._has_hidden_nodes = False

        self._scene_items = {}
        self._graph_nodes_list = []
        self._connections = {}
        self._animations_waiting = set()

        self._grid_size = 36

        self._brush = QtGui.QBrush(convert_hex_to_color(COLOR_PALETTE['background']),
                                   QtCore.Qt.SolidPattern)

        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['grid_line']), 0)

        self._right_click_menu = QtWidgets.QMenu()

        self._show_hidden_nodes = False
        self._hidden_node_action = self.create_bool_right_click_action(
            'Show Hidden Nodes', self._show_hidden_nodes, self.hidden_nodes_toggle)
        self._right_click_menu.addAction(self._hidden_node_action)

        self._show_hidden_topics = False
        self._hidden_topics_action = self.create_bool_right_click_action(
            'Show default hidden topics', self._show_hidden_topics, self.hidden_topics_toggle)
        self._right_click_menu.addAction(self._hidden_topics_action)

        self._show_hidden_services = False
        self._hidden_services_action = self.create_bool_right_click_action(
            'Show default hidden services',
            self._show_hidden_services,
            self.hidden_services_toggle)
        self._right_click_menu.addAction(self._hidden_services_action)

        self._live_updates = True
        self._live_updates_action = self.create_bool_right_click_action(
            'Update scene as graph changes', self._live_updates, self.live_updates_toggle)
        self._right_click_menu.addAction(self._live_updates_action)

        self.newNodeParams.connect(self.update_node_params)
        self.newLifecycleState.connect(self.update_lifecycle_state)
        self.newComponentManagerNodes.connect(self.update_component_manager_nodes)

        self.newNodes.connect(self.update_nodes)

    # PyQt method override
    def drawBackground(self, painter, rect):
        painter.fillRect(rect, self._brush)

        left_line = rect.left() - rect.left() % self._grid_size
        top_line = rect.top() - rect.top() % self._grid_size
        lines = []

        i = int(left_line)
        while i < int(rect.right()):
            lines.append(QtCore.QLineF(i, rect.top(), i, rect.bottom()))
            i += self._grid_size

        u = int(top_line)
        while u < int(rect.bottom()):
            lines.append(QtCore.QLineF(rect.left(), u, rect.right(), u))
            u += self._grid_size

        painter.setPen(self._pen)
        painter.drawLines(lines)

    # PyQt method override
    def contextMenuEvent(self, event):
        self._right_click_menu.popup(QtGui.QCursor.pos())
        super().contextMenuEvent(event)

    def hidden_nodes_toggle(self, checked):
        self._show_hidden_nodes = checked
        # We only do an update if there happen to be hidden nodes in the network
        if self._has_hidden_nodes:
            self.update_nodes(self._graph_nodes_list)

    def hidden_topics_toggle(self, checked):
        self._show_hidden_topics = checked
        self.update_nodes(self._graph_nodes_list)

    def hidden_services_toggle(self, checked):
        self._show_hidden_services = checked
        self.update_nodes(self._graph_nodes_list)

    def live_updates_toggle(self, checked):
        self._live_updates = checked
        self.update_nodes(self._graph_nodes_list)

    def update_nodes(self, new_nodes):
        self._graph_nodes_list = new_nodes

        if not self._live_updates:
            return

        networkx_node_graph = networkx.MultiGraph()
        added_item = False
        items_to_remove = dict(self._scene_items)
        conns_to_remove = dict(self._connections)
        connection_tuples = []

        for node in self._graph_nodes_list:
            should_remove = False
            if node.name.startswith('/_'):
                self._has_hidden_nodes = True
                if not self._show_hidden_nodes:
                    should_remove = True

            if should_remove:
                continue

            if node.name not in self._scene_items:
                self._scene_items[node.name] = NodeBox(node.name,
                                                       node.is_lifecycle,
                                                       node.is_component_manager)
                self.addItem(self._scene_items[node.name])
                added_item = True

            if node.name in items_to_remove:
                del items_to_remove[node.name]
            networkx_node_graph.add_node(node.name)

            for topic in node.topic_publishers + node.topic_subscribers:
                if not self._show_hidden_topics and \
                       topic_is_hidden(topic.conn_name, topic.conn_type):
                    if topic.conn_name in self._scene_items:
                        self.removeItem(self._scene_items[topic.conn_name])
                        del self._scene_items[topic.conn_name]
                    continue

                if topic.conn_name not in self._scene_items:
                    added_item = True
                    new_box = ConnectionBox(topic.conn_name, topic.conn_type, 'topic')
                    self._scene_items[topic.conn_name] = new_box
                    self.addItem(self._scene_items[topic.conn_name])

                if topic.conn_name in items_to_remove:
                    del items_to_remove[topic.conn_name]

                networkx_node_graph.add_edge(node.name, topic.conn_name)
                connection_tuples.append((node.name, topic))

            for service in node.service_clients + node.service_servers:
                if not self._show_hidden_services and \
                       service_is_hidden(service.conn_name, service.conn_type):
                    if service.conn_name in self._scene_items:
                        self.removeItem(self._scene_items[service.conn_name])
                        del self._scene_items[service.conn_name]
                    continue

                if service.conn_name not in self._scene_items:
                    added_item = True
                    new_box = ConnectionBox(service.conn_name, service.conn_type, 'service')
                    self._scene_items[service.conn_name] = new_box
                    self.addItem(self._scene_items[service.conn_name])

                if service.conn_name in items_to_remove:
                    del items_to_remove[service.conn_name]

                networkx_node_graph.add_edge(node.name, service.conn_name)
                connection_tuples.append((node.name, service))

            for action in node.action_clients + node.action_servers:
                if action.conn_name not in self._scene_items:
                    added_item = True
                    new_box = ConnectionBox(action.conn_name, action.conn_type, 'action')
                    self._scene_items[action.conn_name] = new_box
                    self.addItem(self._scene_items[action.conn_name])
                    self._scene_items[action.conn_name].setPos(1100, 900)

                if action.conn_name in items_to_remove:
                    del items_to_remove[action.conn_name]

                networkx_node_graph.add_edge(node.name, action.conn_name)
                connection_tuples.append((node.name, action))

        for name, item in items_to_remove.items():
            if name in self._scene_items:
                self.removeItem(self._scene_items[name])
                del self._scene_items[name]

        for conn_tuple in connection_tuples:
            if conn_tuple not in self._connections:
                added_item = True
                self._connections[conn_tuple] = ConnectionLine(conn_tuple[1].qos_profile)
                self.addItem(self._connections[conn_tuple])
            if conn_tuple in conns_to_remove:
                del conns_to_remove[conn_tuple]

        for conn_tuple, item in conns_to_remove.items():
            self.removeItem(self._connections[conn_tuple])
            del self._connections[conn_tuple]

        if (added_item or items_to_remove) and \
           (networkx_node_graph.nodes or networkx_node_graph.edges):
            # TODO(clalancette): These hard-coded values aren't very good
            pos = networkx.kamada_kawai_layout(networkx_node_graph,
                                               center=(999.0, 999.0),
                                               scale=500.0)

            for name, item in self._scene_items.items():
                if name in pos:
                    self._animations_waiting.add(name)
                    item.set_position(pos[name][0], pos[name][1])

        if not self._animations_waiting:
            self.update_connections()

        self.update()

    def update_connections(self):
        for (from_node, to_conn), item in self._connections.items():
            if from_node in self._scene_items and to_conn.conn_name in self._scene_items:
                item.update_path(self._scene_items[from_node].right(),
                                 self._scene_items[to_conn.conn_name].left())

    def update_node_params(self, node_name, new_params):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_params(new_params)

    def update_lifecycle_state(self, node_name, new_state):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_lifecycle_state(new_state)

    def update_component_manager_nodes(self, node_name, managed_nodes):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_managed_nodes(managed_nodes)

    def complete_animation(self, name):
        if name in self._animations_waiting:
            self._animations_waiting.remove(name)
        self.update_connections()
        self.update()


class NodeGraphicsView(QtWidgets.QGraphicsView):

    def __init__(self, parent, ros_network):
        super().__init__(parent)

        # Hold onto a reference to the ROS 2 network
        self._ros_network = ros_network
        self._node_parameters = {}
        self._lifecycle_states = {}
        self._component_manager_nodes = {}

        self._node_list = []

        # Setup Qt
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        self.setRenderHint(QtGui.QPainter.HighQualityAntialiasing, True)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        self.setRenderHint(QtGui.QPainter.NonCosmeticDefaultPen, True)
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.FullViewportUpdate)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self.setWindowTitle('ROS 2 network visualizer')

        # Instantiate the scene
        scene = NetworkScene(self)
        # TODO(clalancette): Make this configurable or resizeable somehow?
        scene.setSceneRect(0, 0, 2000, 2000)
        self.setScene(scene)

        self.setMouseTracking(True)

        self.selection_box = QtWidgets.QRubberBand(QtWidgets.QRubberBand.Rectangle, self)
        self.selection_origin = None

        # TODO(clalancette): This sucks that we are polling to get changes to
        # the ROS graph.  Unfortunately the graph APIs are not available in
        # Python, so this is the best we can do for now.
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.get_ros_graph_updates)
        self._timer.start(500)

    # PyQt method override
    def wheelEvent(self, event):
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)

        inFactor = 1.15
        outFactor = 1 / inFactor

        if event.angleDelta().y() > 0:
            zoomFactor = inFactor
        else:
            zoomFactor = outFactor

        self.scale(zoomFactor, zoomFactor)

        super().wheelEvent(event)

    # PyQt method override
    def mousePressEvent(self, event):
        if (event.button() == QtCore.Qt.LeftButton and
                event.modifiers() == QtCore.Qt.NoModifier and
                self.scene().itemAt(self.mapToScene(event.pos()), QtGui.QTransform()) is None):
            self.selection_origin = event.pos()
            self.selection_box.setGeometry(QtCore.QRect(event.pos(), QtCore.QSize()))
            self.selection_box.show()

        super().mousePressEvent(event)

    # PyQt method override
    def mouseMoveEvent(self, event):
        if self.selection_origin is not None:
            rect = QtCore.QRect(self.selection_origin, event.pos()).normalized()
            self.selection_box.setGeometry(rect)

        super().mouseMoveEvent(event)

    # PyQt method override
    def mouseReleaseEvent(self, event):
        if self.selection_origin is not None:
            rect = QtCore.QRect(self.selection_origin, event.pos()).normalized()
            self.selection_box.setGeometry(rect)

            painter_path = QtGui.QPainterPath()
            rect = self.mapToScene(self.selection_box.geometry())
            painter_path.addPolygon(rect)
            self.selection_box.hide()
            self.scene().setSelectionArea(painter_path)
            self.selection_origin = None

        super().mouseReleaseEvent(event)

    def update_parameters(self, node_name):
        if node_name not in self._node_parameters:
            self._node_parameters[node_name] = {}

        ret = self._ros_network.get_node_parameters(node_name)
        if ret and ret != self._node_parameters[node_name]:
            self.scene().newNodeParams.emit(node_name, ret)
            self._node_parameters[node_name] = dict(ret)

    def update_lifecycle_state(self, node_name):
        if node_name not in self._lifecycle_states:
            self._lifecycle_states[node_name] = None

        ret = self._ros_network.get_lifecycle_node_state(node_name)
        if ret and ret != self._lifecycle_states[node_name]:
            self.scene().newLifecycleState.emit(node_name, ret)
            self._lifecycle_states[node_name] = ret

    def update_component_manager_nodes(self, node_name):
        if node_name not in self._component_manager_nodes:
            self._component_manager_nodes[node_name] = []

        ret = self._ros_network.get_component_manager_nodes(node_name)
        if ret and ret != self._component_manager_nodes[node_name]:
            self.scene().newComponentManagerNodes.emit(node_name, ret)
            self._component_manager_nodes[node_name] = list(ret)

    def get_ros_graph_updates(self):
        start = time.time()

        node_list = self._ros_network.get_nodes()

        if node_list != self._node_list:
            self._node_list = node_list
            self.scene().newNodes.emit(self._node_list)

            parameters_to_remove = set(self._node_parameters.keys())
            lcs_to_remove = set(self._lifecycle_states.keys())
            cms_to_remove = set(self._component_manager_nodes.keys())

            for node in self._node_list:
                self.update_parameters(node.name)
                if node.is_lifecycle:
                    self.update_lifecycle_state(node.name)
                if node.is_component_manager:
                    self.update_component_manager_nodes(node.name)

                parameters_to_remove.discard(node.name)
                lcs_to_remove.discard(node.name)
                cms_to_remove.discard(node.name)

            for name in parameters_to_remove:
                del self._node_parameters[name]

            for name in lcs_to_remove:
                del self._lifecycle_states[name]

            for name in cms_to_remove:
                del self._component_manager_nodes[name]

        end = time.time()
        print('Timer took %f seconds' % (end - start))

    # PyQt method override
    def closeEvent(self, *args, **kwargs):
        self._ros_network.shutdown()

# TODO(clalancette): Allow the user to click to hide certain connections

# TODO(clalancette): We currently rely on /parameter_events to get updates about
# parameters in the system.  In the case where the user has that disabled, we
# should provide an alternative way to refresh (button, or periodic, maybe?)

# TODO(clalancette): Allow the user to select just groups of nodes to concentrate on


def main():
    app = QtWidgets.QApplication([])

    # Note that we set this up *before* calling rclpy.init()
    # so that we don't override ROS signal handlers
    def do_shutdown(a, b):
        ros_network.shutdown()
        app.quit()
    signal.signal(signal.SIGINT, do_shutdown)

    ros_network = ROSGraph()

    gv = NodeGraphicsView(None, ros_network)
    gv.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
