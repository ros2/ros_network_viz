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
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

import networkx

import rclpy
import rclpy.action
import rclpy.node
import rclpy.topic_or_service_is_hidden
import rcl_interfaces.msg
import rcl_interfaces.srv
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import composition_interfaces.srv


def convert_data_to_color(data):
    # rgb or rgba
    if len(data) in [3, 4]:
        return QtGui.QColor(*data)
    raise Exception('Invalid color, must be list of 3 or 4 items')


DEFAULT_IGNORED_TOPICS = (
    ('/parameter_events', 'rcl_interfaces/msg/ParameterEvent'),
    ('/rosout', 'rcl_interfaces/msg/Log'),
)

DEFAULT_LIFECYCLE_IGNORED_TOPICS = (
    ('/transition_event', 'lifecycle_msgs/msg/TransitionEvent'),
)

DEFAULT_IGNORED_SERVICES = (
    ('/describe_parameters', 'rcl_interfaces/srv/DescribeParameters'),
    ('/get_parameter_types', 'rcl_interfaces/srv/GetParameterTypes'),
    ('/get_parameters', 'rcl_interfaces/srv/GetParameters'),
    ('/list_parameters', 'rcl_interfaces/srv/ListParameters'),
    ('/set_parameters', 'rcl_interfaces/srv/SetParameters'),
    ('/set_parameters_atomically', 'rcl_interfaces/srv/SetParametersAtomically'),
)

DEFAULT_LIFECYCLE_IGNORED_SERVICES = (
    ('/change_state', 'lifecycle_msgs/srv/ChangeState'),
    ('/get_available_states', 'lifecycle_msgs/srv/GetAvailableStates'),
    ('/get_available_transitions', 'lifecycle_msgs/srv/GetAvailableTransitions'),
    ('/get_state', 'lifecycle_msgs/srv/GetState'),
    ('/get_transition_graph', 'lifecycle_msgs/srv/GetAvailableTransitions'),
)


def topic_is_hidden(name, topic_type):
    # First check if rclpy considers this a hidden topic
    if rclpy.topic_or_service_is_hidden.topic_or_service_is_hidden(name):
        return True

    # But then also look through our hard-coded list to skip common topics
    # that don't start with an underscore
    for ignore_topic,ignore_type in DEFAULT_IGNORED_TOPICS:
        if name.startswith(ignore_topic) and ignore_type == topic_type:
            return True

    for ignore_topic,ignore_type in DEFAULT_LIFECYCLE_IGNORED_TOPICS:
        if name.endswith(ignore_topic) and ignore_type == topic_type:
            return True

    return False


def service_is_hidden(name, service_type):
    # First check if rclpy considers this a hidden service
    if rclpy.topic_or_service_is_hidden.topic_or_service_is_hidden(name):
        return True

    # But then also look through our hard-coded list to skip common services
    # that don't start with an underscore
    for ignore_service,ignore_type in DEFAULT_IGNORED_SERVICES + DEFAULT_LIFECYCLE_IGNORED_SERVICES:
        if name.endswith(ignore_service) and ignore_type == service_type:
            return True

    return False


def create_node_namespace(name, namespace):
    return namespace + name if namespace.endswith('/') else namespace + '/' + name


def get_ros_parameter_value(parameter_value):
    if parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == rcl_interfaces.msg.ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        value = None

    return value


class ROSNodeEdge:
    __slots__ = ('from_node', 'to_node', 'connection_name', 'connection_type')

    def __init__(self, from_node, to_node, connection_name, connection_type):
        self.from_node = from_node
        self.to_node = to_node
        self.connection_name = connection_name
        self.connection_type = connection_type

    def __eq__(self, other):
        return self.from_node == other.from_node and self.to_node == other.to_node and self.connection_name == other.connection_name and self.connection_type == other.connection_type

    def __lt__(self, other):
        # We don't really care what the order of the nodes in when we
        # sort, we just care that it is stable.
        return self.from_node < other.from_node and self.to_node < other.to_node and self.connection_name < other.connection_name and self.connection_type < other.connection_type

    def __repr__(self):
        from_print = self.from_node
        if from_print is None:
            from_print = '(nil)'
        to_print = self.to_node
        if to_print is None:
            to_print = '(nil)'
        return from_print + ' --' + self.connection_name + '--> ' + to_print


class ROSAsyncServiceStateMachine:
    NEEDS_CLIENT = 1
    NEEDS_CLIENT_READY = 2
    NEEDS_RESPONSE = 3
    HAS_INITIAL_RESPONSE = 4

    def __init__(self, rosnode, node_name, rostype, create_request_cb,
                 service_name, periodic_refresh_seconds=0.0):
        self._rosnode = rosnode
        self._node_name = node_name
        self._rostype = rostype
        self._create_request_cb = create_request_cb
        self._service_name = service_name
        self._periodic_refresh_seconds = periodic_refresh_seconds

        self._state = self.NEEDS_CLIENT
        self._client = None
        self._future = None
        self._sent_request_time = None
        self._last_update_time = None
        self._result = None

    def step(self):
        if self._state == self.NEEDS_CLIENT:
            self._client = self._rosnode.create_client(self._rostype,
                                                       f'{self._node_name}/{self._service_name}')
            self._state = self.NEEDS_CLIENT_READY

        elif self._state == self.NEEDS_CLIENT_READY:
            if self._client.service_is_ready():
                self._future = self._client.call_async(self._create_request_cb())
                self._sent_request_time = time.time()
                self._state = self.NEEDS_RESPONSE
            # implicit else: continue waiting for the list client to be ready

        elif self._state == self.NEEDS_RESPONSE:
            if self._future.done():
                future_result = self._future.result()
                if future_result is None:
                    self._future = None
                    self._state = self.NEEDS_CLIENT_READY
                else:
                    self._result = future_result
                    self._state = self.HAS_INITIAL_RESPONSE
                    self._last_update_time = time.time()
            else:
                if time.time() - self._sent_request_time > 5.0:
                    # If we heard nothing back in 5 seconds, resend the request
                    self._future = None
                    self._state = self.NEEDS_CLIENT_READY

                # implicit else: continue waiting for a response to the list request

        elif self._state == self.HAS_INITIAL_RESPONSE:
            if self._periodic_refresh_seconds > 0.0:
                if time.time() - self._last_update_time > self._periodic_refresh_seconds:
                    self._state = self.NEEDS_CLIENT_READY
            else:
                if self._client:
                    self._rosnode.destroy_client(self._client)
                    self._client = None

        return self._result


class ROSParameterStateMachine:
    def __init__(self, rosnode, node_name):
        self._list_state_machine = ROSAsyncServiceStateMachine(rosnode,
                                                               node_name,
                                                               rcl_interfaces.srv.ListParameters,
                                                               self.create_list_params_request,
                                                               'list_parameters')

        self._get_state_machine = ROSAsyncServiceStateMachine(rosnode,
                                                              node_name,
                                                              rcl_interfaces.srv.GetParameters,
                                                              self.create_get_params_request,
                                                              'get_parameters')

        self._param_names = []

        self._parameters = {}
        self._parameters_lock = threading.Lock()

    def create_list_params_request(self):
        return rcl_interfaces.srv.ListParameters.Request()

    def create_get_params_request(self):
        req = rcl_interfaces.srv.GetParameters.Request()
        req.names = self._param_names
        return req

    def step(self):
        list_param_response = self._list_state_machine.step()
        if self._list_state_machine._state == ROSAsyncServiceStateMachine.HAS_INITIAL_RESPONSE and list_param_response:
            self._param_names = list_param_response.result.names

            get_params_response = self._get_state_machine.step()
            if self._get_state_machine._state == ROSAsyncServiceStateMachine.HAS_INITIAL_RESPONSE and get_params_response:
                self._parameters_lock.acquire()
                # TODO(clalancette): what happens if the get_params_response is shorter than the parameter names?
                for i, name in enumerate(list_param_response.result.names):
                    if not name in self._parameters:
                        self._parameters[name] = get_ros_parameter_value(get_params_response.values[i])
                self._parameters_lock.release()

        return self._parameters

    def update_parameters_from_msg(self, msg):
        self._parameters_lock.acquire()
        for param in msg.new_parameters:
            self._parameters[param.name] = get_ros_parameter_value(param.value)
        for param in msg.changed_parameters:
            self._parameters[param.name] = get_ros_parameter_value(param.value)
        for param in msg.deleted_parameters:
            del self._parameters[param.name]
        self._parameters_lock.release()


class ROSLifecycleStateMachine:
    def __init__(self, rosnode, node_name):
        # TODO(clalancette): We are never destroying this
        self._lifecycle_transition_sub = rosnode.create_subscription(lifecycle_msgs.msg.TransitionEvent,
                                                                     node_name + '/transition_event',
                                                                     self.lifecycle_transition_cb,
                                                                     10)

        self._lifecycle_state_machine = ROSAsyncServiceStateMachine(rosnode,
                                                                    node_name,
                                                                    lifecycle_msgs.srv.GetState,
                                                                    self.create_lifecycle_request,
                                                                    'get_state')

        self._lifecycle_state = None
        self._lifecycle_state_lock = threading.Lock()

    def create_lifecycle_request(self):
        return lifecycle_msgs.srv.GetState.Request()

    def step(self):
        lifecycle_state_response = self._lifecycle_state_machine.step()
        if self._lifecycle_state_machine._state == ROSAsyncServiceStateMachine.HAS_INITIAL_RESPONSE and lifecycle_state_response:
            self._lifecycle_state_lock.acquire()
            if self._lifecycle_state is None:
                self._lifecycle_state = lifecycle_state_response.current_state.label
            self._lifecycle_state_lock.release()

        return self._lifecycle_state

    def lifecycle_transition_cb(self, msg):
        self._lifecycle_state_lock.acquire()
        self._lifecycle_state = msg.goal_state.label
        self._lifecycle_state_lock.release()


class ROSComponentManagerListNodesStateMachine:
    def __init__(self, rosnode, node_name):

        # TODO(clalancette): Having to use periodic refresh sucks; it puts a lot
        # more load on the the network.  But there isn't a notification
        # mechanism for when the list of nodes changes
        self._component_state_machine = ROSAsyncServiceStateMachine(rosnode,
                                                                    node_name,
                                                                    composition_interfaces.srv.ListNodes,
                                                                    self.create_list_nodes_request,
                                                                    '_container/list_nodes', 5.0)

        self._nodes = []

    def create_list_nodes_request(self):
        return composition_interfaces.srv.ListNodes.Request()

    def step(self):
        list_nodes_response = self._component_state_machine.step()
        if self._component_state_machine._state == ROSAsyncServiceStateMachine.HAS_INITIAL_RESPONSE and list_nodes_response:
            self._nodes = list_nodes_response.full_node_names

        return self._nodes


class ROSNetwork:
    def __init__(self):
        self._node = rclpy.create_node('rqt_network')

        self._param_events_sub = self._node.create_subscription(rcl_interfaces.msg.ParameterEvent,
                                                                '/parameter_events',
                                                                self.parameter_event_cb,
                                                                10)

        self._param_clients = {}

        self._lc_state_clients = {}
        self._component_manager_nodes_clients = {}

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._executor.spin)
        self._spin_thread.start()

    def get_topic_edges(self, name, namespace):
        edges = []

        node_full_name = create_node_namespace(name, namespace)

        for topic_name, topic_types in self._node.get_publisher_names_and_types_by_node(name, namespace):
            had_subscriber = False
            for topic_info in self._node.get_subscriptions_info_by_topic(topic_name):
                sub_node_full_name = create_node_namespace(topic_info.node_name, topic_info.node_namespace)
                e = ROSNodeEdge(node_full_name, sub_node_full_name, topic_name, topic_info.topic_type)
                if node_full_name != sub_node_full_name and e not in edges:
                    edges.append(e)
                    had_subscriber = True

            if not had_subscriber:
                for topic_type in topic_types:
                    edges.append(ROSNodeEdge(node_full_name, None, topic_name, topic_type))

        for topic_name, topic_types in self._node.get_subscriber_names_and_types_by_node(name, namespace):
            had_publisher = False
            for topic_info in self._node.get_publishers_info_by_topic(topic_name):
                pub_node_full_name = create_node_namespace(topic_info.node_name, topic_info.node_namespace)
                e = ROSNodeEdge(pub_node_full_name, node_full_name, topic_name, topic_info.topic_type)
                if node_full_name != pub_node_full_name and e not in edges:
                    edges.append(e)
                    had_publisher = True

            if not had_publisher:
                for topic_type in topic_types:
                    edges.append(ROSNodeEdge(None, node_full_name, topic_name, topic_type))

        return edges

    def get_service_edges(self, name, namespace, node_to_lifecycle, node_to_component_manager):
        edges = []

        node_full_name = create_node_namespace(name, namespace)

        if not node_full_name in node_to_lifecycle:
            node_to_lifecycle[node_full_name] = False

        if not node_full_name in node_to_component_manager:
            node_to_component_manager[node_full_name] = False

        for service_name, service_types in self._node.get_service_names_and_types_by_node(name, namespace):
            for service_type in service_types:
                edges.append(ROSNodeEdge(None, node_full_name, service_name, service_type))
                if 'get_state' in service_name and service_type == 'lifecycle_msgs/srv/GetState':
                    node_to_lifecycle[node_full_name] = True

                if 'list_nodes' in service_name and service_type == 'composition_interfaces/srv/ListNodes':
                    node_to_component_manager[node_full_name] = True

        return edges

    def get_action_edges(self, name, namespace):
        edges = []

        node_full_name = create_node_namespace(name, namespace)

        for action_name, action_types in rclpy.action.get_action_server_names_and_types_by_node(self._node, name, namespace):
            for action_type in action_types:
                edges.append(ROSNodeEdge(None, node_full_name, action_name, action_type))

        return edges

    def get_edges(self):
        topic_edges = []
        service_edges = []
        action_edges = []
        node_to_lifecycle = {}
        node_to_component_manager = {}

        for name, namespace in self._node.get_node_names_and_namespaces():
            try:
                topic_edges.extend(self.get_topic_edges(name, namespace))

                service_edges.extend(self.get_service_edges(name, namespace, node_to_lifecycle, node_to_component_manager))

                action_edges.extend(self.get_action_edges(name, namespace))

            except rclpy.node.NodeNameNonExistentError:
                # It's possible that the node exited between when we saw it
                # in get_node_names_and_namespaces() and now.  If that happens,
                # just skip the processing.
                pass

        for name, namespace in self._node.get_node_names_and_namespaces():
            node_full_name = create_node_namespace(name, namespace)

            try:
                for service_name, service_types in self._node.get_client_names_and_types_by_node(name, namespace):
                    for service_type in service_types:
                        client_had_server = False
                        for e in service_edges:
                            if service_name == e.connection_name and service_type == e.connection_type:
                                e.from_node = node_full_name
                                client_had_server = True

                        if not client_had_server:
                            service_edges.append(ROSNodeEdge(node_full_name, None, service_name, service_type))

                for action_name, action_types in rclpy.action.get_action_client_names_and_types_by_node(self._node, name, namespace):
                    for action_type in action_types:
                        action_client_had_server = False
                        for e in action_edges:
                            if action_name == e.connection_name and action_type == e.connection_type:
                                e.from_node = node_full_name
                                action_client_had_server = True

                        if not action_client_had_server:
                            action_edges.append(ROSNodeEdge(node_full_name, None, action_name, action_type))

            except rclpy.node.NodeNameNonExistentError:
                # It's possible that the node exited between when we saw it
                # in get_node_names_and_namespaces() and now.  If that happens,
                # just skip the processing.
                pass

        return topic_edges, service_edges, action_edges, node_to_lifecycle, node_to_component_manager

    def parameter_event_cb(self, msg):
        if msg.node in self._param_clients:
            self._param_clients[msg.node].update_parameters_from_msg(msg)

    def get_node_parameters(self, node_name):
        # This method is useful to get all of the parameters and values from a
        # a node.  It is meant to be entirely non-blocking, which means that if
        # any part of the pipeline isn't ready, this will return an empty
        # dictionary.  Subsequent calls may then get the data.

        if node_name not in self._param_clients:
            self._param_clients[node_name] = ROSParameterStateMachine(self._node, node_name)

        return self._param_clients[node_name].step()

    def get_lifecycle_node_state(self, node_name):
        # This method is useful to get the lifecycle state of a Lifecycle node.
        # It should *only* be called on a Lifecycle node, but this method does
        # not enforce that.  Calling it on a non-Lifecycle node is not fatal,
        # but will eat up valuable resources in the network.  It is meant to be
        # entirely non-blocking, which means that if any part of the pipeline
        # isn't ready, this will return None.  Subsequent calls may get the data.

        if node_name not in self._lc_state_clients:
            self._lc_state_clients[node_name] = ROSLifecycleStateMachine(self._node, node_name)

        return self._lc_state_clients[node_name].step()

    def get_component_manager_nodes(self, node_name):
        # This method is useful to get the nodes that are part of a Component
        # Manager.  It should *only* be called on a Component Manager node, but
        # this method does not enforce that.  Calling it on a non-Component
        # Manager node is not fatal, but will eat up valuable resources in the
        # network.  It is meant to be entirely non-blocking, which means that
        # if any part of the pipeline isn't ready, this will return None.
        # Subsequent calls may get the data.

        if node_name not in self._component_manager_nodes_clients:
            self._component_manager_nodes_clients[node_name] = ROSComponentManagerListNodesStateMachine(self._node, node_name)

        return self._component_manager_nodes_clients[node_name].step()

    def shutdown(self):
        self._node.destroy_subscription(self._param_events_sub)
        # TODO(clalancette): We really should call destroy_node() here, but it
        # can race with calls to get_{topic,service}_edges()
        # self._node.destroy_node()
        self._executor.shutdown()
        self._spin_thread.join()


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
    def __init__(self, parent, ros_network):
        super().__init__(parent)

        # Hold onto a reference to the ROS 2 network
        self._ros_network = ros_network
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

    rclpy.init()
    network = ROSNetwork()

    gv = NodeGraphicsView(None, network)
    gv.show()

    return app.exec_()

if __name__ == '__main__':
    sys.exit(main())
