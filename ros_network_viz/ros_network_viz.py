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

from PyQt5 import QtCore, QtGui, QtWidgets

import networkx

from ros_network_viz.ros_graph import ROSGraph, topic_is_hidden, service_is_hidden

def convert_data_to_color(data):
    # rgb or rgba
    if len(data) in [3, 4]:
        return QtGui.QColor(*data)
    raise Exception('Invalid color, must be list of 3 or 4 items')


class ConnectionItem(QtWidgets.QGraphicsPathItem):
    def __init__(self):
        super().__init__()

        self.setZValue(-1)

        self._pen = QtGui.QPen(convert_data_to_color([255, 155, 0, 255]))
        self._pen.setWidth(2)

        self.setPen(self._pen)

    def update_path(self, source_point, target_point):
        path = QtGui.QPainterPath()

        path.moveTo(source_point)
        dx = (target_point.x() - source_point.x()) * 0.5
        dy = target_point.y() - source_point.y()
        ctrl1 = QtCore.QPointF(source_point.x() + dx, source_point.y() + dy * 0)
        ctrl2 = QtCore.QPointF(source_point.x() + dx, source_point.y() + dy * 1)
        path.cubicTo(ctrl1, ctrl2, target_point)

        self.setPath(path)


class AttrItem(QtWidgets.QGraphicsItem):
    def __init__(self, parent, name, index):
        super().__init__(parent)

        self._name = name
        self._index = index

        self._has_right_ellipse = False
        self._has_left_ellipse = False

        self._bg_pen = QtGui.QPen()
        self._bg_pen.setStyle(QtCore.Qt.SolidLine)
        self._bg_pen.setColor(convert_data_to_color([0, 0, 0, 0]))

        self._bg_brush = QtGui.QBrush()
        self._bg_brush.setStyle(QtCore.Qt.SolidPattern)
        self._bg_brush.setColor(convert_data_to_color([60, 60, 60, 255]))

        self._ellipse_pen = QtGui.QPen()
        self._ellipse_pen.setStyle(QtCore.Qt.SolidLine)

        self._right_ellipse_brush = QtGui.QBrush()
        self._right_ellipse_brush.setStyle(QtCore.Qt.SolidPattern)
        self._right_ellipse_brush.setColor(convert_data_to_color([255, 155, 0, 255]))

        self._left_ellipse_brush = QtGui.QBrush()
        self._left_ellipse_brush.setStyle(QtCore.Qt.SolidPattern)
        self._left_ellipse_brush.setColor(convert_data_to_color([0, 155, 255, 255]))

        self._text_font = QtGui.QFont('Arial', 10, QtGui.QFont.Normal)
        self._text_pen = QtGui.QPen()
        self._text_pen.setColor(convert_data_to_color([220, 220, 220, 255]))

    # PyQt method override
    def boundingRect(self):
        parent = self.parentItem()

        ellipse_width = parent._attr_height / 2.0

        if self._has_left_ellipse:
            x = -ellipse_width / 2.0
        else:
            x = 0

        if self._has_right_ellipse:
            width = parent.width() + ellipse_width / 2.0
        else:
            width = parent.width()

        y = parent._base_height - parent._radius + self._index * parent._attr_height

        return QtCore.QRectF(x, y, width, parent._attr_height)

    # PyQt method override
    def paint(self, painter, option, widget):
        parent = self.parentItem()

        # First draw the background
        rect = QtCore.QRect(int(parent._node_border / 2),
                            parent._base_height - parent._radius + self._index * parent._attr_height,
                            parent.width() - parent._node_border,
                            parent._attr_height)
        painter.setPen(self._bg_pen)
        painter.setBrush(self._bg_brush)

        painter.drawRect(rect)

        # Now draw the text
        painter.setPen(self._text_pen)
        painter.setFont(self._text_font)

        text_rect = QtCore.QRect(rect.left() + parent._radius,
                                 rect.top(),
                                 rect.width() - 2 * parent._radius,
                                 rect.height())
        painter.drawText(text_rect, QtCore.Qt.AlignVCenter, self._name)

        # Now draw the ellipses
        painter.setPen(self._ellipse_pen)

        if self._has_right_ellipse:
            painter.setBrush(self._right_ellipse_brush)
            painter.drawEllipse(self.ellipseRect(True))

        if self._has_left_ellipse:
            painter.setBrush(self._left_ellipse_brush)
            painter.drawEllipse(self.ellipseRect(False))

    def width(self):
        metrics = QtGui.QFontMetrics(self._text_font)
        return metrics.boundingRect(self._name).width() + 14

    def ellipseRect(self, is_right):
        parent = self.parentItem()

        width = parent._attr_height / 2.0
        height = parent._attr_height / 2.0

        if is_right:
            x = parent.width() - (width / 2.0)
        else:
            x = -width / 2.0

        y = parent._base_height - parent._radius + parent._attr_height / 4 + self._index * parent._attr_height

        return QtCore.QRectF(x, y, width, height)

    def center(self, is_publisher):
        rect = self.ellipseRect(is_publisher)
        center = QtCore.QPointF(rect.x() + rect.width() * 0.5,
                                rect.y() + rect.height() * 0.5)

        return self.mapToScene(center)


class TopicAttrItem(AttrItem):
    def __init__(self, parent, name, index):
        super().__init__(parent, name, index)

        self._right_ellipse_brush.setColor(convert_data_to_color([255, 155, 0, 255]))
        self._left_ellipse_brush.setColor(convert_data_to_color([0, 155, 255, 255]))
        self._bg_brush.setColor(convert_data_to_color([60, 60, 60, 255]))

    def set_has_publisher(self, has_pub):
        self._has_right_ellipse = has_pub

    def set_has_subscriber(self, has_sub):
        self._has_left_ellipse = has_sub

    def get_has_publisher(self):
        return self._has_right_ellipse

    def get_has_subscriber(self):
        return self._has_left_ellipse


class ServiceAttrItem(AttrItem):
    def __init__(self, parent, name, index):
        super().__init__(parent, name, index)

        self._right_ellipse_brush.setColor(convert_data_to_color([255, 255, 255, 255]))
        self._left_ellipse_brush.setColor(convert_data_to_color([0, 0, 0, 255]))
        self._bg_brush.setColor(convert_data_to_color([160, 160, 160, 255]))

    def set_has_client(self, has_client):
        self._has_right_ellipse = has_client

    def set_has_server(self, has_server):
        self._has_left_ellipse = has_server

    def get_has_client(self):
        return self._has_right_ellipse

    def get_has_server(self):
        return self._has_left_ellipse


class ActionAttrItem(AttrItem):
    def __init__(self, parent, name, index):
        super().__init__(parent, name, index)

        self._right_ellipse_brush.setColor(convert_data_to_color([25, 25, 25, 255]))
        self._left_ellipse_brush.setColor(convert_data_to_color([130, 130, 130, 255]))
        self._bg_brush.setColor(convert_data_to_color([250, 120, 120, 255]))

    def set_has_client(self, has_client):
        self._has_right_ellipse = has_client

    def set_has_server(self, has_server):
        self._has_left_ellipse = has_server

    def get_has_client(self):
        return self._has_right_ellipse

    def get_has_server(self):
        return self._has_left_ellipse


class NodeItem(QtWidgets.QGraphicsObject):
    def __init__(self, name):
        super().__init__()

        self._name = name

        self._lifecycle_state = None
        self._topic_attrs = {}
        self._service_attrs = {}
        self._action_attrs = {}
        self._params = {}
        self._managed_nodes = []

        self._attr_height = 30
        self._radius = 10
        self._base_height = 25
        self._base_width = 200
        self._node_border = 2

        self.setZValue(1)
        self.setAcceptHoverEvents(True)

        self.setFlag(QtWidgets.QGraphicsObject.ItemIsMovable)
        self.setFlag(QtWidgets.QGraphicsObject.ItemIsSelectable)

        self._brush = QtGui.QBrush()
        self._brush.setStyle(QtCore.Qt.SolidPattern)
        self._brush.setColor(convert_data_to_color([80, 80, 80, 255]))

        self._pen = QtGui.QPen()
        self._pen.setStyle(QtCore.Qt.SolidLine)
        self._pen.setWidth(self._node_border)
        self._pen.setColor(convert_data_to_color([50, 50, 50, 255]))

        self._pen_sel = QtGui.QPen()
        self._pen_sel.setStyle(QtCore.Qt.SolidLine)
        self._pen_sel.setWidth(self._node_border)
        self._pen_sel.setColor(convert_data_to_color([170, 80, 80, 255]))

        self._text_pen = QtGui.QPen()
        self._text_pen.setStyle(QtCore.Qt.SolidLine)
        self._text_pen.setColor(convert_data_to_color([230, 230, 230, 255]))

        self._node_text_font = QtGui.QFont('Arial', 12, QtGui.QFont.Bold)

        self._node_info_box = None

        self._anim = QtCore.QPropertyAnimation(self, b'pos')

    # PyQt method override
    def boundingRect(self):
        return QtCore.QRectF(QtCore.QRect(0, 0, self.width(), self.height()))

    # PyQt method override
    def paint(self, painter, option, widget):
        # First draw the rounded rectangle the represents the node
        painter.setBrush(self._brush)
        if self.isSelected():
            painter.setPen(self._pen_sel)
        else:
            painter.setPen(self._pen)

        painter.drawRoundedRect(0, 0,
                                self.width(),
                                self.height(),
                                self._radius,
                                self._radius)

        # Now draw the node name (and lifecycle state, if applicable) above the
        # rectangle
        painter.setPen(self._text_pen)
        painter.setFont(self._node_text_font)

        name = self._name
        if self._lifecycle_state is not None:
            name += ' (' + self._lifecycle_state + ')'

        metrics = QtGui.QFontMetrics(painter.font())
        text_width = metrics.boundingRect(name).width() + 14
        text_height = metrics.boundingRect(name).height() + 14
        margin = int((text_width - self.width()) * 0.5)
        text_rect = QtCore.QRect(-margin,
                                -text_height,
                                text_width,
                                text_height)

        painter.drawText(text_rect,
                         QtCore.Qt.AlignCenter,
                         name)

        # TODO(clalancette): This info box never really gets destroyed, and
        # seems to "flicker" a lot on screen.  Also it is always rendered
        # directly in the middle of the node, rather than following the mouse.
        # Can we make this nicer?
        if self._node_info_box is not None:
            rect = self.boundingRect()
            center = QtCore.QPointF(rect.x() + rect.width() * 0.5,
                                    rect.y() + rect.height() * 0.5)

            self._node_info_box.setPos(self.mapToScene(center))

    # PyQt method override
    def mouseMoveEvent(self, event):
        self.scene().update_connections()
        super().mouseMoveEvent(event)

    # PyQt method override
    def hoverEnterEvent(self, event):
        if self._params:
            text = 'Parameters:\n'
            for k,v in self._params.items():
                text += '  ' + k + ' -> ' + str(v) + '\n'
            text = text[:-1]
        else:
            # TODO(clalancette): Maybe show the difference between there are no
            # actual parameters, and the fact that we can't fetch them
            text = 'No Parameters'

        label = QtWidgets.QLabel(text)
        label_proxy = QtWidgets.QGraphicsProxyWidget()
        label_proxy.setWidget(label)

        layout = QtWidgets.QGraphicsLinearLayout(QtCore.Qt.Vertical)
        layout.addItem(label_proxy)

        self._node_info_box = QtWidgets.QGraphicsWidget()
        self._node_info_box.setZValue(2)
        self._node_info_box.setLayout(layout)
        self.scene().addItem(self._node_info_box)

        super().hoverEnterEvent(event)

    # PyQt method override
    def hoverLeaveEvent(self, event):
        if self._node_info_box:
            self.scene().removeItem(self._node_info_box)
        super().hoverLeaveEvent(event)

    def width(self):
        ret = self._base_width

        for name,item in self._topic_attrs.items():
            width = item.width()
            if width > ret:
                ret = width

        for name,item in self._service_attrs.items():
            width = item.width()
            if width > ret:
                ret = width

        return ret

    def height(self):
        ret = self._base_height
        num_attrs = len(self._topic_attrs) + len(self._service_attrs)
        if num_attrs > 0:
            ret = ret + self._attr_height * num_attrs + self._node_border + 0.5 * self._radius

        return int(ret)

    def reindex_attributes(self):
        index = 0
        for attr_name, item in self._topic_attrs.items():
            item._index = index
            index += 1

        for attr_name, item in self._service_attrs.items():
            item._index = index
            index += 1

        for attr_name, item in self._action_attrs.items():
            item._index = index
            index += 1

    def add_topic_attribute(self, attr_name, is_publisher):
        if attr_name not in self._topic_attrs:
            self._topic_attrs[attr_name] = TopicAttrItem(self, attr_name, len(self._topic_attrs) + len(self._service_attrs))

        if is_publisher:
            self._topic_attrs[attr_name].set_has_publisher(True)
        else:
            self._topic_attrs[attr_name].set_has_subscriber(True)

        self.reindex_attributes()

    def get_topic_attribute(self, attr_name):
        if attr_name in self._topic_attrs:
            return self._topic_attrs[attr_name]
        return None

    def remove_topic_attribute(self, attr_name, is_publisher):
        if not attr_name in self._topic_attrs:
            return

        if is_publisher:
            self._topic_attrs[attr_name].set_has_publisher(False)
        else:
            self._topic_attrs[attr_name].set_has_subscriber(False)

        if not self._topic_attrs[attr_name].get_has_publisher() and not self._topic_attrs[attr_name].get_has_subscriber():
            self._topic_attrs[attr_name].setVisible(False)
            del self._topic_attrs[attr_name]

        self.reindex_attributes()

    def add_service_attribute(self, attr_name, is_client):
        if attr_name not in self._service_attrs:
            self._service_attrs[attr_name] = ServiceAttrItem(self, attr_name, len(self._topic_attrs) + len(self._service_attrs))

        if is_client:
            self._service_attrs[attr_name].set_has_client(True)
        else:
            self._service_attrs[attr_name].set_has_server(True)

        self.reindex_attributes()

    def get_service_attribute(self, attr_name):
        if attr_name in self._service_attrs:
            return self._service_attrs[attr_name]
        return None

    def remove_service_attribute(self, attr_name, is_client):
        if not attr_name in self._service_attrs:
            return

        if is_client:
            self._service_attrs[attr_name].set_has_client(False)
        else:
            self._service_attrs[attr_name].set_has_server(False)

        if not self._service_attrs[attr_name].get_has_client() and not self._service_attrs[attr_name].get_has_server():
            self._service_attrs[attr_name].setVisible(False)
            del self._service_attrs[attr_name]

        self.reindex_attributes()

    def add_action_attribute(self, attr_name, is_client):
        if attr_name not in self._action_attrs:
            self._action_attrs[attr_name] = ActionAttrItem(self, attr_name, len(self._topic_attrs) + len(self._action_attrs))

        if is_client:
            self._action_attrs[attr_name].set_has_client(True)
        else:
            self._action_attrs[attr_name].set_has_server(True)

        self.reindex_attributes()

    def get_action_attribute(self, attr_name):
        if attr_name in self._action_attrs:
            return self._action_attrs[attr_name]
        return None

    def remove_action_attribute(self, attr_name, is_client):
        if not attr_name in self._action_attrs:
            return

        if is_client:
            self._action_attrs[attr_name].set_has_client(False)
        else:
            self._action_attrs[attr_name].set_has_server(False)

        if not self._action_attrs[attr_name].get_has_client() and not self._action_attrs[attr_name].get_has_server():
            self._action_attrs[attr_name].setVisible(False)
            del self._action_attrs[attr_name]

        self.reindex_attributes()

    def update_params(self, new_params):
        self._params = new_params

    def update_lifecycle_state(self, new_state):
        self._lifecycle_state = new_state
        self._brush.setColor(convert_data_to_color([25, 130, 25, 255]))

    def update_managed_nodes(self, managed_nodes):
        self._managed_nodes = managed_nodes
        self._brush.setColor(convert_data_to_color([130, 25, 25, 255]))

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


class NetworkScene(QtWidgets.QGraphicsScene):
    new_graph_signal = QtCore.pyqtSignal(list, list, list, name='newGraph')
    new_node_params_signal = QtCore.pyqtSignal(str, dict, name='newNodeParams')
    new_lifecycle_state_signal = QtCore.pyqtSignal(str, str, name='newLifecycleState')
    new_component_manager_nodes_signal = QtCore.pyqtSignal(str, list, name='newComponentManagerNodes')

    def __init__(self, parent):
        super().__init__(parent)

        self._connections = {}
        self._nodes = {}
        self._topic_edges = []
        self._service_edges = []
        self._action_edges = []
        self._has_hidden_nodes = False
        self._animations_waiting = set()

        self._grid_size = 36

        self._brush = QtGui.QBrush()
        self._brush.setStyle(QtCore.Qt.SolidPattern)
        self._brush.setColor(convert_data_to_color([40, 40,40, 255]))

        self._pen = QtGui.QPen()
        self._pen.setColor(convert_data_to_color([50, 50, 50, 255]))
        self._pen.setWidth(0)

        self._right_click_menu = QtWidgets.QMenu()

        self._show_hidden_nodes = False
        self._hidden_node_action = QtWidgets.QAction('Show Hidden Nodes', self)
        self._hidden_node_action.setCheckable(True)
        self._hidden_node_action.setChecked(self._show_hidden_nodes)
        self._hidden_node_action.triggered.connect(self.hidden_nodes_toggle)
        self._right_click_menu.addAction(self._hidden_node_action)

        self._show_rqt_network_node = False
        self._hidden_rqt_network_action = QtWidgets.QAction('Show rqt_network node', self)
        self._hidden_rqt_network_action.setCheckable(True)
        self._hidden_rqt_network_action.setChecked(self._show_rqt_network_node)
        self._hidden_rqt_network_action.triggered.connect(self.hidden_rqt_network_toggle)
        self._right_click_menu.addAction(self._hidden_rqt_network_action)

        self._show_hidden_topics = False
        self._hidden_topics_action = QtWidgets.QAction('Show default hidden topics', self)
        self._hidden_topics_action.setCheckable(True)
        self._hidden_topics_action.setChecked(self._show_hidden_topics)
        self._hidden_topics_action.triggered.connect(self.hidden_topics_toggle)
        self._right_click_menu.addAction(self._hidden_topics_action)

        self._show_hidden_services = False
        self._hidden_services_action = QtWidgets.QAction('Show default hidden services', self)
        self._hidden_services_action.setCheckable(True)
        self._hidden_services_action.setChecked(self._show_hidden_services)
        self._hidden_services_action.triggered.connect(self.hidden_services_toggle)
        self._right_click_menu.addAction(self._hidden_services_action)

        self._live_updates = True
        self._live_updates_action = QtWidgets.QAction('Update scene as graph changes')
        self._live_updates_action.setCheckable(True)
        self._live_updates_action.setChecked(self._live_updates)
        self._live_updates_action.triggered.connect(self.live_updates_toggle)
        self._right_click_menu.addAction(self._live_updates_action)

        self.newGraph.connect(self.update_edges)
        self.newNodeParams.connect(self.update_node_params)
        self.newLifecycleState.connect(self.update_lifecycle_state)
        self.newComponentManagerNodes.connect(self.update_component_manager_nodes)

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
            self.update_edges(self._topic_edges, self._service_edges, self._action_edges)

    def hidden_rqt_network_toggle(self, checked):
        self._show_rqt_network_node = checked
        self.update_edges(self._topic_edges, self._service_edges, self._action_edges)

    def hidden_topics_toggle(self, checked):
        self._show_hidden_topics = checked
        self.update_edges(self._topic_edges, self._service_edges, self._action_edges)

    def hidden_services_toggle(self, checked):
        self._show_hidden_services = checked
        self.update_edges(self._topic_edges, self._service_edges, self._action_edges)

    def live_updates_toggle(self, checked):
        self._live_updates = checked
        self.update_edges(self._topic_edges, self._service_edges, self._action_edges)

    def apply_node_options(self, from_node, to_node):
        if from_node is not None and from_node.startswith('/_'):
            self._has_hidden_nodes = True
            if not self._show_hidden_nodes:
                from_node = None

        if to_node is not None and to_node.startswith('/_'):
            self._has_hidden_nodes = True
            if not self._show_hidden_nodes:
                to_node = None

        if not self._show_rqt_network_node:
            if from_node == '/rqt_network':
                from_node = None

            if to_node == '/rqt_network':
                to_node = None

        return (from_node, to_node)

    def add_node(self, node_name):
        if node_name not in self._nodes:
            self._nodes[node_name] = NodeItem(node_name)
            self.addItem(self._nodes[node_name])
            return True

        return False

    def add_topic(self, node_name, full_topic_name, is_publisher, visible_topic):
        if visible_topic:
            self._nodes[node_name].add_topic_attribute(full_topic_name, is_publisher)
        else:
            self._nodes[node_name].remove_topic_attribute(full_topic_name, is_publisher)

    def add_service(self, node_name, full_service_name, is_client, visible_service):
        if visible_service:
            self._nodes[node_name].add_service_attribute(full_service_name, is_client)
        else:
            self._nodes[node_name].remove_service_attribute(full_service_name, is_client)

    def add_action(self, node_name, full_action_name, is_client):
        self._nodes[node_name].add_action_attribute(full_action_name, is_client)

    def update_edges(self, topic_edges, service_edges, action_edges):
        self._topic_edges = topic_edges
        self._service_edges = service_edges
        self._action_edges = action_edges

        if not self._live_updates:
            return

        self._has_hidden_nodes = False
        networkx_node_graph = networkx.MultiGraph()
        added_node = False
        nodes_to_remove = dict(self._nodes)
        conns_to_remove = dict(self._connections)
        for e in self._topic_edges:
            (from_node, to_node) = self.apply_node_options(e.from_node, e.to_node)

            if from_node is None and to_node is None:
                continue

            networkx_node_graph.add_edges_from([(from_node, to_node)])

            full_topic_name = e.connection_name + ': ' + e.connection_type
            visible_topic = self._show_hidden_topics or not topic_is_hidden(e.connection_name, e.connection_type)

            if from_node is not None:
                added_node = self.add_node(from_node) or added_node
                self.add_topic(from_node, full_topic_name, True, visible_topic)
                if from_node in nodes_to_remove:
                    del nodes_to_remove[from_node]

            if to_node is not None:
                added_node = self.add_node(to_node) or added_node
                self.add_topic(to_node, full_topic_name, False, visible_topic)
                if to_node in nodes_to_remove:
                    del nodes_to_remove[to_node]

            if visible_topic:
                if from_node is not None and to_node is not None:
                    conn_tuple = (from_node, full_topic_name, to_node)
                    if conn_tuple not in self._connections:
                        self._connections[conn_tuple] = ConnectionItem()
                        self.addItem(self._connections[conn_tuple])
                    if conn_tuple in conns_to_remove:
                        del conns_to_remove[conn_tuple]

        for e in self._service_edges:
            (from_node, to_node) = self.apply_node_options(e.from_node, e.to_node)

            if from_node is None and to_node is None:
                continue

            networkx_node_graph.add_edges_from([(from_node, to_node)])

            full_service_name = e.connection_name + ': ' + e.connection_type
            visible_service = self._show_hidden_services or not service_is_hidden(e.connection_name, e.connection_type)

            if from_node is not None:
                added_node = self.add_node(from_node) or added_node
                self.add_service(from_node, full_service_name, True, visible_service)
                if from_node in nodes_to_remove:
                    del nodes_to_remove[from_node]

            if to_node is not None:
                added_node = self.add_node(to_node) or added_node
                self.add_service(to_node, full_service_name, False, visible_service)
                if to_node in nodes_to_remove:
                    del nodes_to_remove[to_node]

            if visible_service:
                if from_node is not None and to_node is not None:
                    conn_tuple = (from_node, full_service_name, to_node)
                    if conn_tuple not in self._connections:
                        self._connections[conn_tuple] = ConnectionItem()
                        self.addItem(self._connections[conn_tuple])
                    if conn_tuple in conns_to_remove:
                        del conns_to_remove[conn_tuple]

        for e in self._action_edges:
            (from_node, to_node) = self.apply_node_options(e.from_node, e.to_node)

            if from_node is None and to_node is None:
                continue

            networkx_node_graph.add_edges_from([(from_node, to_node)])

            full_action_name = e.connection_name + ': ' + e.connection_type

            if from_node is not None:
                added_node = self.add_node(from_node) or added_node
                self.add_action(from_node, full_action_name, True)
                if from_node in nodes_to_remove:
                    del nodes_to_remove[from_node]

            if to_node is not None:
                added_node = self.add_node(to_node) or added_node
                self.add_action(to_node, full_action_name, False)
                if to_node in nodes_to_remove:
                    del nodes_to_remove[to_node]

            if from_node is not None and to_node is not None:
                conn_tuple = (from_node, full_action_name, to_node)
                if conn_tuple not in self._connections:
                    self._connections[conn_tuple] = ConnectionItem()
                    self.addItem(self._connections[conn_tuple])
                if conn_tuple in conns_to_remove:
                    del conns_to_remove[conn_tuple]

        for name, item in nodes_to_remove.items():
            self.removeItem(self._nodes[name])
            del self._nodes[name]

        for conn_tuple, item in conns_to_remove.items():
            self.removeItem(self._connections[conn_tuple])
            del self._connections[conn_tuple]

        if added_node or nodes_to_remove:
            # TODO(clalancette): These hard-coded values aren't very good
            pos = networkx.spring_layout(networkx_node_graph, center=(999.0, 999.0), scale=300.0, k=300.0)

            for name, item in self._nodes.items():
                self._animations_waiting.add(name)
                item.set_position(pos[name][0], pos[name][1])

        if not self._animations_waiting:
            self.update_connections()

        self.update()

    def update_connections(self):
        for (from_node, connection_name, to_node), item in self._connections.items():
            source_attr = self._nodes[from_node].get_topic_attribute(connection_name)
            target_attr = self._nodes[to_node].get_topic_attribute(connection_name)
            if source_attr is not None and target_attr is not None:
                if source_attr.get_has_publisher() and target_attr.get_has_subscriber():
                    item.update_path(source_attr.center(True), target_attr.center(False))
                continue

            source_attr = self._nodes[from_node].get_service_attribute(connection_name)
            target_attr = self._nodes[to_node].get_service_attribute(connection_name)
            if source_attr is not None and target_attr is not None:
                if source_attr.get_has_client() and target_attr.get_has_server():
                    item.update_path(source_attr.center(True), target_attr.center(False))

    def update_node_params(self, node_name, new_params):
        if node_name in self._nodes:
            self._nodes[node_name].update_params(new_params)

    def update_lifecycle_state(self, node_name, new_state):
        if node_name in self._nodes:
            self._nodes[node_name].update_lifecycle_state(new_state)

    def update_component_manager_nodes(self, node_name, managed_nodes):
        if node_name in self._nodes:
            self._nodes[node_name].update_managed_nodes(managed_nodes)

    def complete_animation(self, name):
        if name in self._animations_waiting:
            self._animations_waiting.remove(name)
        self.update_connections()
        self.update()


class NodeGraphicsView(QtWidgets.QGraphicsView):
    def __init__(self, parent):
        super().__init__(parent)

        # Hold onto a reference to the ROS 2 network
        self._ros_network = ROSGraph()
        self._topic_edges = []
        self._service_edges = []
        self._action_edges = []
        self._node_parameters = {}
        self._lifecycle_states = {}
        self._component_manager_nodes = {}

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
        # TODO(clalancette): Make this configurable or resizeable somehow?
        scene.setSceneRect(0, 0, 2000, 2000)
        self.setScene(scene)

        self.setMouseTracking(True)

        # TODO(clalancette): This sucks that we are polling to get changes to
        # the ROS graph.  Unfortunately the graph APIs are not available in
        # Python, so this is the best we can do for now.
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.get_new_edges)
        self._timer.start(500)

    def update_parameters(self, node_name):
        if node_name is None:
            return

        if node_name not in self._node_parameters:
            self._node_parameters[node_name] = {}

        ret = self._ros_network.get_node_parameters(node_name)
        if ret and ret != self._node_parameters[node_name]:
            self.scene().newNodeParams.emit(node_name, ret)
            self._node_parameters[node_name] = ret

    def update_lifecycle_state(self, node_name):
        if node_name not in self._lifecycle_states:
            self._lifecycle_states[node_name] = None

        ret = self._ros_network.get_lifecycle_node_state(node_name)
        if ret and ret != self._lifecycle_states[node_name]:
            self.scene().newLifecycleState.emit(node_name, ret)
            self._lifecycle_states[node_name] = ret

    def update_component_manager_nodes(self, node_name):
        if node_name not in self._component_manager_nodes:
            self._component_manager_nodes[node_name] = None

        ret = self._ros_network.get_component_manager_nodes(node_name)
        if ret and ret != self._component_manager_nodes[node_name]:
            self.scene().newComponentManagerNodes.emit(node_name, ret)
            self._component_manager_nodes[node_name] = ret

    def get_new_edges(self):
        start = time.time()
        topic_edges, service_edges, action_edges, node_to_lifecycle, node_to_component_manager = self._ros_network.get_edges()
        if topic_edges != self._topic_edges or service_edges != self._service_edges or action_edges != self._action_edges:
            self._topic_edges = topic_edges
            self._service_edges = service_edges
            self._action_edges = action_edges
            self.scene().newGraph.emit(self._topic_edges, self._service_edges, self._action_edges)

        for edge in topic_edges:
            self.update_parameters(edge.from_node)
            self.update_parameters(edge.to_node)

        for edge in service_edges:
            if edge.from_node is not None:
                if node_to_lifecycle[edge.from_node]:
                    self.update_lifecycle_state(edge.from_node)
                if node_to_component_manager[edge.from_node]:
                    self.update_component_manager_nodes(edge.from_node)

            if edge.to_node is not None:
                if node_to_lifecycle[edge.to_node]:
                    self.update_lifecycle_state(edge.to_node)
                if node_to_component_manager[edge.to_node]:
                    self.update_component_manager_nodes(edge.to_node)

        end = time.time()
        print('Timer took %f seconds' % (end - start))

    # PyQt method override
    def closeEvent(self, *args, **kwargs):
        self._ros_network.shutdown()

# TODO(clalancette): Consider a box for topics, like in rqt_graph.  This may
# reduce the number of lines on the graph with many publishers and subscribers

# TODO(clalancette): Allow the user to click to hide certain connections

# TODO(clalancette): We currently rely on /parameter_events to get updates about
# parameters in the system.  In the case where the user has that disabled, we
# should provide an alternative way to refresh (button, or periodic, maybe?)

# TODO(clalancette): Show QoS parameters for topics

# TODO(clalancette): Allow the user to select just groups of nodes to concentrate on

# TODO(clalancette): Show nodes that have no topics

def main():
    app = QtWidgets.QApplication([])

    # Note that we set this up *before* calling rclpy.init()
    # so that we don't override ROS signal handlers
    def do_shutdown(a, b):
        network.shutdown()
        app.quit()
    signal.signal(signal.SIGINT, do_shutdown)

    gv = NodeGraphicsView(None)
    gv.show()

    return app.exec_()

if __name__ == '__main__':
    sys.exit(main())
