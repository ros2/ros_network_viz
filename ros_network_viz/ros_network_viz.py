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

from ros_network_viz.ros_graph import node_is_hidden, ROSGraph, service_is_hidden, topic_is_hidden
from ros_network_viz.sorted_ordered_set import SortedOrderedSet

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

        # Pens used to draw the connection line in selected and unselected states
        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['connection_line']), 3)
        self._pen_sel = QtGui.QPen(
            convert_hex_to_color(COLOR_PALETTE['connection_line_highlight']), 5)

        # Brushes used to fill the arrow in selected and unselected states
        self._brush = QtGui.QBrush(convert_hex_to_color(COLOR_PALETTE['connection_line']))
        self._brush_sel = QtGui.QBrush(convert_hex_to_color(COLOR_PALETTE['connection_line_highlight']))

        self.setPen(self._pen)

        # The connection line begins in the unselected state
        self._current_brush = self._brush

        # The endpoints of the connection line aren't known until update_path is called
        self._arrow_location = None

    # PyQt method override
    def hoverEnterEvent(self, event):
        self.setPen(self._pen_sel)
        self._current_brush = self._brush_sel
        super().hoverEnterEvent(event)

    # PyQt method override
    def hoverLeaveEvent(self, event):
        self.setPen(self._pen)
        self._current_brush = self._brush
        super().hoverLeaveEvent(event)

    # PyQt method override
    def paint(self, painter, option, widget):
        super().paint(painter, option, widget)

        # update_path() may not have been called yet
        if self._arrow_location:
            x = self._arrow_location.x()
            y = self._arrow_location.y()
            path = QtGui.QPainterPath()
            path.moveTo(x - 15, y + 10)
            path.lineTo(x - 15, y - 10)
            path.lineTo(x + 1, y)
            path.moveTo(x - 15, y + 10)
            painter.fillPath(path, self._current_brush)

    def update_path(self, source_point, target_point):
        path = QtGui.QPainterPath()

        self._arrow_location = target_point
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
        self._param_warnings = ''
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
        self._anim.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._anim.setDuration(1000)
        self._anim.finished.connect(self.animation_finished)

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
            text += 'Lifecycle state: ' + self._lifecycle_state + '\n'

        if self._managed_nodes:
            text += 'Managed Nodes:\n'
            for n in self._managed_nodes:
                text += '  ' + n + '\n'

        text += 'Parameters:\n'
        if self._param_warnings:
            text += '  <' + self._param_warnings + '>\n'
        elif self._params:
            for k, v in self._params.items():
                text += '  ' + k + ' -> ' + str(v) + '\n'
        else:
            text += '  None\n'

        text = text[:-1]

        self.setToolTip(text)

    def update_params(self, new_params, new_warnings):
        self._params = new_params
        self._param_warnings = new_warnings
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
        self._anim.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._anim.setDuration(1000)
        self._anim.finished.connect(self.animation_finished)

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
        self._anim.start()

    def left(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(0, rect.y() + rect.height() * 0.5))

    def right(self):
        rect = self.boundingRect()
        return self.mapToScene(QtCore.QPointF(rect.x() + rect.width(),
                                              rect.y() + rect.height() * 0.5))


class NetworkScene(QtWidgets.QGraphicsScene):

    new_node_params_signal = QtCore.pyqtSignal(str, dict, str, name='newNodeParams')
    new_lifecycle_state_signal = QtCore.pyqtSignal(str, str, name='newLifecycleState')
    new_component_nodes_signal = QtCore.pyqtSignal(str, list, name='newComponentManagerNodes')

    new_nodes_signal = QtCore.pyqtSignal(dict, name='newNodes')

    update_hidden_nodes_signal = QtCore.pyqtSignal(list, name='updateHiddenNodes')
    update_hidden_topics_signal = QtCore.pyqtSignal(list, name='updateHiddenTopics')
    update_hidden_services_signal = QtCore.pyqtSignal(list, name='updateHiddenServices')
    update_hidden_actions_signal = QtCore.pyqtSignal(list, name='updateHiddenActions')

    def create_bool_right_click_action(self, name, boolean_variable, callback):
        action = QtWidgets.QAction(name, self)
        action.setCheckable(True)
        action.setChecked(boolean_variable)
        action.triggered.connect(callback)
        return action

    def __init__(self, parent):
        super().__init__(parent)

        self._scene_items = {}
        self._graph_nodes_list = {}
        self._connections = {}
        self._animations_waiting = set()
        self._hidden_nodes = set()
        self._hidden_topics = set()
        self._hidden_services = set()
        self._hidden_actions = set()

        self._grid_size = 36

        self._brush = QtGui.QBrush(convert_hex_to_color(COLOR_PALETTE['background']),
                                   QtCore.Qt.SolidPattern)

        self._pen = QtGui.QPen(convert_hex_to_color(COLOR_PALETTE['grid_line']), 0)

        self._right_click_menu = QtWidgets.QMenu()

        self._live_updates = True
        self._live_updates_action = self.create_bool_right_click_action(
            'Update scene as graph changes', self._live_updates, self.live_updates_toggle)
        self._right_click_menu.addAction(self._live_updates_action)

        self.newNodeParams.connect(self.update_node_params)
        self.newLifecycleState.connect(self.update_lifecycle_state)
        self.newComponentManagerNodes.connect(self.update_component_manager_nodes)

        self.newNodes.connect(self.update_nodes)
        self.updateHiddenNodes.connect(self.update_hidden_nodes)
        self.updateHiddenTopics.connect(self.update_hidden_topics)
        self.updateHiddenServices.connect(self.update_hidden_services)
        self.updateHiddenActions.connect(self.update_hidden_actions)

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

        for name, node in self._graph_nodes_list.items():
            if node.name not in self._scene_items:
                self._scene_items[node.name] = NodeBox(node.name,
                                                       node.is_lifecycle,
                                                       node.is_component_manager)
                self.addItem(self._scene_items[node.name])
                added_item = True

            if node.name in items_to_remove:
                del items_to_remove[node.name]

            if node.name in self._hidden_nodes:
                self._scene_items[node.name].setVisible(False)
                continue
            else:
                self._scene_items[node.name].setVisible(True)

            networkx_node_graph.add_node(node.name)

            def update_one_topic(node_name, topic, direction):
                added_topic = False

                # TODO(clalancette): What if there are multiple topics with
                # different types and/or QoS settings?
                if topic.conn_name in self._hidden_topics:
                    if topic.conn_name in self._scene_items:
                        self._scene_items[topic.conn_name].setVisible(False)
                    return added_topic

                if topic.conn_name not in self._scene_items:
                    added_topic = True
                    new_box = ConnectionBox(topic.conn_name, topic.conn_type, 'topic')
                    self._scene_items[topic.conn_name] = new_box
                    self.addItem(self._scene_items[topic.conn_name])

                self._scene_items[topic.conn_name].setVisible(True)

                if topic.conn_name in items_to_remove:
                    del items_to_remove[topic.conn_name]

                if direction == 'node_to_conn':
                    networkx_node_graph.add_edge(node_name, topic.conn_name)
                else:
                    networkx_node_graph.add_edge(topic.conn_name, node_name)

                connection_tuples.append((node_name, topic, direction))

                return added_topic

            def update_one_service(node_name, service, direction):
                added_service = False

                # TODO(clalancette): What if there are multiple services with
                # different types and/or QoS settings?
                if service.conn_name in self._hidden_services:
                    if service.conn_name in self._scene_items:
                        self._scene_items[service.conn_name].setVisible(False)
                    return added_service

                if service.conn_name not in self._scene_items:
                    added_service = True
                    new_box = ConnectionBox(service.conn_name, service.conn_type, 'service')
                    self._scene_items[service.conn_name] = new_box
                    self.addItem(self._scene_items[service.conn_name])

                self._scene_items[service.conn_name].setVisible(True)

                if service.conn_name in items_to_remove:
                    del items_to_remove[service.conn_name]

                if direction == 'node_to_conn':
                    networkx_node_graph.add_edge(node_name, service.conn_name)
                else:
                    networkx_node_graph.add_edge(service.conn_name, node_name)

                connection_tuples.append((node_name, service, direction))

                return added_service

            def update_one_action(node_name, action, direction):
                added_action = False

                if action.conn_name in self._hidden_actions:
                    if action.conn_name in self._scene_items:
                        self._scene_items[action.conn_name].setVisible(False)
                    return added_action

                if action.conn_name not in self._scene_items:
                    added_action = True
                    new_box = ConnectionBox(action.conn_name, action.conn_type, 'action')
                    self._scene_items[action.conn_name] = new_box
                    self.addItem(self._scene_items[action.conn_name])
                    self._scene_items[action.conn_name].setPos(1100, 900)

                self._scene_items[action.conn_name].setVisible(True)

                if action.conn_name in items_to_remove:
                    del items_to_remove[action.conn_name]

                if direction == 'node_to_conn':
                    networkx_node_graph.add_edge(node_name, action.conn_name)
                else:
                    networkx_node_graph.add_edge(action.conn_name, node_name)

                connection_tuples.append((node_name, action, direction))

                return added_action

            for topic in node.topic_publishers:
                added_item = update_one_topic(node.name, topic, 'node_to_conn')

            for topic in node.topic_subscribers:
                added_item = update_one_topic(node.name, topic, 'conn_to_node')

            for service in node.service_clients:
                added_item = update_one_service(node.name, service, 'node_to_conn')

            for service in node.service_servers:
                added_item = update_one_service(node.name, service, 'conn_to_node')

            for action in node.action_clients:
                added_item = update_one_action(node.name, action, 'node_to_conn')

            for action in node.action_servers:
                added_item = update_one_action(node.name, action, 'conn_to_node')

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
            center_x = self.parent()._size.width() // 2
            center_y = self.parent()._size.height() // 2
            pos = networkx.kamada_kawai_layout(networkx_node_graph,
                                               center=(center_x, center_y),
                                               scale=400.0)

            for name, item in self._scene_items.items():
                if name in pos:
                    self._animations_waiting.add(name)
                    item.set_position(pos[name][0], pos[name][1])

        if not self._animations_waiting:
            self.update_connections()

        self.update()

    def update_connections(self):
        for (node_name, conn, direction), item in self._connections.items():
            if node_name in self._scene_items and conn.conn_name in self._scene_items:
                if direction == 'node_to_conn':
                    source = self._scene_items[node_name].right()
                    target = self._scene_items[conn.conn_name].left()
                else:
                    source = self._scene_items[conn.conn_name].right()
                    target = self._scene_items[node_name].left()

                item.update_path(source, target)

    def update_node_params(self, node_name, new_params, new_warnings):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_params(new_params, new_warnings)

    def update_lifecycle_state(self, node_name, new_state):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_lifecycle_state(new_state)

    def update_component_manager_nodes(self, node_name, managed_nodes):
        if node_name in self._scene_items:
            self._scene_items[node_name].update_managed_nodes(managed_nodes)

    def update_hidden_nodes(self, node_names):
        before = set(self._hidden_nodes)

        for node_name, is_visible in node_names:
            if is_visible:
                self._hidden_nodes.discard(node_name)
            else:
                self._hidden_nodes.add(node_name)

        if before != self._hidden_nodes:
            self.update_nodes(self._graph_nodes_list)

    def update_hidden_topics(self, topic_names):
        before = set(self._hidden_topics)

        for topic_name, is_visible in topic_names:
            if is_visible:
                self._hidden_topics.discard(topic_name)
            else:
                self._hidden_topics.add(topic_name)

        if before != self._hidden_topics:
            self.update_nodes(self._graph_nodes_list)

    def update_hidden_services(self, service_names):
        before = set(self._hidden_services)

        for service_name, is_visible in service_names:
            if is_visible:
                self._hidden_services.discard(service_name)
            else:
                self._hidden_services.add(service_name)

        if before != self._hidden_services:
            self.update_nodes(self._graph_nodes_list)

    def update_hidden_actions(self, action_names):
        before = set(self._hidden_actions)

        for action_name, is_visible in action_names:
            if is_visible:
                self._hidden_actions.discard(action_name)
            else:
                self._hidden_actions.add(action_name)

        if before != self._hidden_actions:
            self.update_nodes(self._graph_nodes_list)

    def complete_animation(self, name):
        if name in self._animations_waiting:
            self._animations_waiting.remove(name)
        self.update_connections()
        self.update()


class NodeGraphicsView(QtWidgets.QGraphicsView):

    def __init__(self, parent):
        super().__init__(parent)

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

        # Instantiate the scene
        scene = NetworkScene(self)
        self.setScene(scene)

        self.setMouseTracking(True)

        self.selection_box = QtWidgets.QRubberBand(QtWidgets.QRubberBand.Rectangle, self)
        self.selection_origin = None

        self._size = QtCore.QSize(500, 500)

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

    # PyQt method override
    def resizeEvent(self, event):
        self._size = event.size()
        return super().resizeEvent(event)

    # PyQt method override
    def sizeHint(self):
        return self._size


class MainGrid(QtWidgets.QWidget):

    def __init__(self, ros_network):
        super().__init__()

        self._ros_network = ros_network
        self._node_parameters = {}
        self._lifecycle_states = {}
        self._component_manager_nodes = {}
        self._node_list = {}
        self._node_name_to_row = SortedOrderedSet()
        self._topic_name_to_row = SortedOrderedSet()
        self._service_name_to_row = SortedOrderedSet()
        self._action_name_to_row = SortedOrderedSet()

        self.setWindowTitle('ROS 2 network visualizer')

        self._splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        self._gv = NodeGraphicsView(self)

        self._model = QtGui.QStandardItemModel(self)
        self._model.setRowCount(0)

        self._labels_to_rows = {
            'Nodes': 0,
            'Topics': 1,
            'Services': 2,
            'Actions': 3,
        }

        label = ''
        for name in self._labels_to_rows.keys():
            self._model.invisibleRootItem().appendRow([self.create_checkable_item(name, False)])
            if label:
                label = label + ', '
            label = label + name
        self._model.setHorizontalHeaderLabels([label])
        self._model.itemChanged.connect(self.state_changed)

        self._tree = QtWidgets.QTreeView(self)
        self._tree.setModel(self._model)
        self._tree.expandAll()

        self._splitter.addWidget(self._gv)
        self._splitter.addWidget(self._tree)

        self._hlayout = QtWidgets.QHBoxLayout()
        self._hlayout.addWidget(self._splitter)

        self.setLayout(self._hlayout)

        # TODO(clalancette): This sucks that we are polling to get changes to
        # the ROS graph.  Unfortunately the graph APIs are not available in
        # Python, so this is the best we can do for now.
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.get_ros_graph_updates)
        self._timer.start(500)

    # PyQt method override
    def closeEvent(self, *args, **kwargs):
        self._ros_network.shutdown()

    def state_changed(self, item):
        parent = item.parent()
        if parent is None:
            item_type = item.text()
        else:
            item_type = parent.text()
        item_name = item.text()
        item_state = item.checkState()

        item_list = []

        if parent is not None:
            # If the parent is not None, that means this is an individual
            # check-box, and we {en,dis}able just that item
            item_list.append((item_name, item_state == QtCore.Qt.Checked))
        else:
            # If the parent is None, this is a top-level check-box and we want
            # to {en,dis}able everything

            is_visible = item_state == QtCore.Qt.Checked

            this_row = self._model.invisibleRootItem().child(self._labels_to_rows[item_name])
            # If we leave the itemChanged signal in place, then Qt emits a
            # itemChanged signal for *every* item we change the check state for,
            # which can cause us to do many redraws and hold up the UI.  Instead
            # we disconnect the signal temporarily, mark all of the check boxes,
            # and then reenable the signal.  Then below we emit one large change
            # which is a lot faster.
            self._model.itemChanged.disconnect(self.state_changed)
            for i in range(0, this_row.rowCount()):
                this_row.child(i).setCheckState(item_state)
                item_list.append((this_row.child(i).text(), is_visible))
            self._model.itemChanged.connect(self.state_changed)

        if item_type == 'Nodes':
            self._gv.scene().updateHiddenNodes.emit(item_list)
        elif item_type == 'Topics':
            self._gv.scene().updateHiddenTopics.emit(item_list)
        elif item_type == 'Services':
            self._gv.scene().updateHiddenServices.emit(item_list)
        elif item_type == 'Actions':
            self._gv.scene().updateHiddenActions.emit(item_list)

    def create_checkable_item(self, name, is_hidden):
        item = QtGui.QStandardItem(name)
        item.setCheckable(True)
        if not is_hidden:
            item.setCheckState(QtCore.Qt.Checked)
        return item

    def update_nodes(self):
        node_list = self._ros_network.get_nodes()

        if node_list == self._node_list:
            return

        # Here, we assume that we are going to remove *all* of the existing
        # nodes.  As we see node names in the new list, we remove them from
        # this list, which means at the end we are left with just the ones
        # we should remove.
        nodes_to_remove = dict(self._node_list)
        topics_to_remove = self._topic_name_to_row.copy()
        services_to_remove = self._service_name_to_row.copy()
        actions_to_remove = self._action_name_to_row.copy()

        for name, node in node_list.items():
            if name not in self._node_list:
                index = self._node_name_to_row.add(name)
                checked_node = self.create_checkable_item(name, node_is_hidden(name))
                self._model.invisibleRootItem().child(
                    self._labels_to_rows['Nodes']).insertRow(index, checked_node)
                if node_is_hidden(name):
                    self._gv.scene().updateHiddenNodes.emit([(name, False)])
            self._node_list[name] = node

            for topic in node.topic_publishers + node.topic_subscribers:
                if topic.conn_name not in self._topic_name_to_row:
                    index = self._topic_name_to_row.add(topic.conn_name)
                    hidden = topic_is_hidden(topic.conn_name, topic.conn_type)
                    checked_topic = self.create_checkable_item(topic.conn_name, hidden)
                    self._model.invisibleRootItem().child(
                        self._labels_to_rows['Topics']).insertRow(index, checked_topic)
                    if hidden:
                        self._gv.scene().updateHiddenTopics.emit([(topic.conn_name, False)])

                if topic.conn_name in topics_to_remove:
                    topics_to_remove.discard(topic.conn_name)

            for service in node.service_clients + node.service_servers:
                if service.conn_name not in self._service_name_to_row:
                    index = self._service_name_to_row.add(service.conn_name)
                    hidden = service_is_hidden(service.conn_name, service.conn_type)
                    checked_service = self.create_checkable_item(service.conn_name, hidden)
                    self._model.invisibleRootItem().child(
                        self._labels_to_rows['Services']).insertRow(index, checked_service)
                    if hidden:
                        self._gv.scene().updateHiddenServices.emit([(service.conn_name, False)])

                if service.conn_name in services_to_remove:
                    services_to_remove.discard(service.conn_name)

            for action in node.action_clients + node.action_servers:
                if action.conn_name not in self._action_name_to_row:
                    index = self._action_name_to_row.add(action.conn_name)
                    checked_action = self.create_checkable_item(action.conn_name, False)
                    self._model.invisibleRootItem().child(
                        self._labels_to_rows['Actions']).insertRow(index, checked_action)

                if action.conn_name in actions_to_remove:
                    actions_to_remove.discard(action.conn_name)

            if name in nodes_to_remove:
                del nodes_to_remove[name]

        for name, node in nodes_to_remove.items():
            if name in self._node_list:
                del self._node_list[name]
            if name in self._node_parameters:
                del self._node_parameters[name]
            if name in self._lifecycle_states:
                del self._lifecycle_states[name]
            if name in self._component_manager_nodes:
                del self._component_manager_nodes[name]

            if name in self._node_name_to_row:
                found_row = self._node_name_to_row.index(name)
                child = self._model.invisibleRootItem().child(self._labels_to_rows['Nodes'])
                child.removeRow(found_row)
                self._node_name_to_row.discard(name)

        for topic in topics_to_remove:
            if topic in self._topic_name_to_row:
                found_row = self._topic_name_to_row.index(topic)
                child = self._model.invisibleRootItem().child(self._labels_to_rows['Topics'])
                child.removeRow(found_row)
                self._topic_name_to_row.discard(topic)

        for service in services_to_remove:
            if service in self._service_name_to_row:
                found_row = self._service_name_to_row.index(service)
                child = self._model.invisibleRootItem().child(self._labels_to_rows['Services'])
                child.removeRow(found_row)
                self._service_name_to_row.discard(service)

        for action in actions_to_remove:
            if action in self._action_name_to_row:
                found_row = self._action_name_to_row.index(action)
                child = self._model.invisibleRootItem().child(self._labels_to_rows['Actions'])
                child.removeRow(found_row)
                self._action_name_to_row.discard(action)

        self._gv.scene().newNodes.emit(self._node_list)

    def update_parameters(self, node_name):
        if node_name not in self._node_parameters:
            self._node_parameters[node_name] = ({}, '')

        params, warnings = self._ros_network.get_node_parameters(node_name)
        if (params and params != self._node_parameters[node_name][0]) or \
           (warnings and warnings != self._node_parameters[node_name][1]):
            self._gv.scene().newNodeParams.emit(node_name, params, warnings)
            self._node_parameters[node_name] = (dict(params), warnings)

    def update_lifecycle_state(self, node_name):
        if node_name not in self._lifecycle_states:
            self._lifecycle_states[node_name] = None

        ret = self._ros_network.get_lifecycle_node_state(node_name)
        if ret and ret != self._lifecycle_states[node_name]:
            self._gv.scene().newLifecycleState.emit(node_name, ret)
            self._lifecycle_states[node_name] = ret

    def update_component_manager_nodes(self, node_name):
        if node_name not in self._component_manager_nodes:
            self._component_manager_nodes[node_name] = []

        ret = self._ros_network.get_component_manager_nodes(node_name)
        if ret and ret != self._component_manager_nodes[node_name]:
            self._gv.scene().newComponentManagerNodes.emit(node_name, ret)
            self._component_manager_nodes[node_name] = list(ret)

    def get_ros_graph_updates(self):
        start = time.time()

        self.update_nodes()

        # TODO(clalancette): We need to do this every time we update so that
        # we keep the state machines running.  But it means that we are
        # iterating over the list twice, once in update_nodes and once here.
        # On the other hand, update_nodes is expensive if the list updates,
        # so we don't necessarily want to include that there.  Not sure what
        # to do about this yet.
        for name, node in self._node_list.items():
            self.update_parameters(name)
            if node.is_lifecycle:
                self.update_lifecycle_state(name)
            if node.is_component_manager:
                self.update_component_manager_nodes(name)

        end = time.time()
        print('Timer took %f seconds' % (end - start))


# TODO(clalancette): We currently rely on /parameter_events to get updates about
# parameters in the system.  In the case where the user has that disabled, we
# should provide an alternative way to refresh (button, or periodic, maybe?)

# TODO(clalancette): It sure would be nice if the connections animated along
# with everything else.

# TODO(clalancette): It would be nice if there was an arrow or other indication
# on whether a connection is a publication, subscription, etc.

# TODO(clalancette): On large graphs, the redraw update can take a long time.
# What's going on with that?


def main():
    app = QtWidgets.QApplication([])

    # Note that we set this up *before* calling rclpy.init()
    # so that we don't override ROS signal handlers
    def do_shutdown(a, b):
        ros_network.shutdown()
        app.quit()
    signal.signal(signal.SIGINT, do_shutdown)

    ros_network = ROSGraph()

    grid = MainGrid(ros_network)
    grid.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
