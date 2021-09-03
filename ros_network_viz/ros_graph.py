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

import threading
import time

import composition_interfaces.srv
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import rclpy.action
import rclpy.node
import rclpy.topic_or_service_is_hidden


IGNORED_TOPICS = (
    ('/parameter_events', 'rcl_interfaces/msg/ParameterEvent'),
    ('/rosout', 'rcl_interfaces/msg/Log'),
)

LIFECYCLE_IGNORED_TOPICS = (
    ('/transition_event', 'lifecycle_msgs/msg/TransitionEvent'),
)

IGNORED_SERVICES = (
    ('/describe_parameters', 'rcl_interfaces/srv/DescribeParameters'),
    ('/get_parameter_types', 'rcl_interfaces/srv/GetParameterTypes'),
    ('/get_parameters', 'rcl_interfaces/srv/GetParameters'),
    ('/list_parameters', 'rcl_interfaces/srv/ListParameters'),
    ('/set_parameters', 'rcl_interfaces/srv/SetParameters'),
    ('/set_parameters_atomically', 'rcl_interfaces/srv/SetParametersAtomically'),
)

LIFECYCLE_IGNORED_SERVICES = (
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
    for ignore_topic, ignore_type in IGNORED_TOPICS:
        if name.startswith(ignore_topic) and ignore_type == topic_type:
            return True

    for ignore_topic, ignore_type in LIFECYCLE_IGNORED_TOPICS:
        if name.endswith(ignore_topic) and ignore_type == topic_type:
            return True

    return False


def service_is_hidden(name, service_type):
    # First check if rclpy considers this a hidden service
    if rclpy.topic_or_service_is_hidden.topic_or_service_is_hidden(name):
        return True

    # But then also look through our hard-coded list to skip common services
    # that don't start with an underscore
    for ignore_service, ignore_type in IGNORED_SERVICES + LIFECYCLE_IGNORED_SERVICES:
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
        return self.from_node == other.from_node and \
            self.to_node == other.to_node and \
            self.connection_name == other.connection_name and \
            self.connection_type == other.connection_type

    def __lt__(self, other):
        # We don't really care what the order of the nodes in when we
        # sort, we just care that it is stable.
        return self.from_node < other.from_node and \
            self.to_node < other.to_node and \
            self.connection_name < other.connection_name and \
            self.connection_type < other.connection_type

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

    def has_initial_response(self):
        return self._state == self.HAS_INITIAL_RESPONSE


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
        if self._list_state_machine.has_initial_response() and list_param_response:
            self._param_names = list_param_response.result.names

            get_params_response = self._get_state_machine.step()
            if self._get_state_machine.has_initial_response() and get_params_response:
                self._parameters_lock.acquire()
                # TODO(clalancette): what happens if the get_params_response is
                # shorter than the parameter names?
                for i, name in enumerate(list_param_response.result.names):
                    if name not in self._parameters:
                        val = get_ros_parameter_value(get_params_response.values[i])
                        self._parameters[name] = val
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
        self._lc_transition_sub = rosnode.create_subscription(lifecycle_msgs.msg.TransitionEvent,
                                                              node_name + '/transition_event',
                                                              self.lifecycle_transition_cb,
                                                              10)

        self._lc_state_machine = ROSAsyncServiceStateMachine(rosnode,
                                                             node_name,
                                                             lifecycle_msgs.srv.GetState,
                                                             self.create_lifecycle_request,
                                                             'get_state')

        self._lifecycle_state = None
        self._lifecycle_state_lock = threading.Lock()

    def create_lifecycle_request(self):
        return lifecycle_msgs.srv.GetState.Request()

    def step(self):
        lifecycle_state_response = self._lc_state_machine.step()
        if self._lc_state_machine.has_initial_response() and lifecycle_state_response:
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
        self._comp_state_machine = ROSAsyncServiceStateMachine(
            rosnode,
            node_name,
            composition_interfaces.srv.ListNodes,
            self.create_list_nodes_request,
            '_container/list_nodes',
            5.0)

        self._nodes = []

    def create_list_nodes_request(self):
        return composition_interfaces.srv.ListNodes.Request()

    def step(self):
        list_nodes_response = self._comp_state_machine.step()
        if self._comp_state_machine.has_initial_response() and list_nodes_response:
            self._nodes = list_nodes_response.full_node_names

        return self._nodes


class ROSGraph:

    def __init__(self):
        rclpy.init()
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

        pubs = self._node.get_publisher_names_and_types_by_node(name, namespace)
        for topic_name, topic_types in pubs:
            had_subscriber = False
            for topic_info in self._node.get_subscriptions_info_by_topic(topic_name):
                sub_node_full_name = create_node_namespace(topic_info.node_name,
                                                           topic_info.node_namespace)
                e = ROSNodeEdge(node_full_name,
                                sub_node_full_name,
                                topic_name,
                                topic_info.topic_type)
                if node_full_name != sub_node_full_name and e not in edges:
                    edges.append(e)
                    had_subscriber = True

            if not had_subscriber:
                for topic_type in topic_types:
                    edges.append(ROSNodeEdge(node_full_name, None, topic_name, topic_type))

        subs = self._node.get_subscriber_names_and_types_by_node(name, namespace)
        for topic_name, topic_types in subs:
            had_publisher = False
            for topic_info in self._node.get_publishers_info_by_topic(topic_name):
                pub_node_full_name = create_node_namespace(topic_info.node_name,
                                                           topic_info.node_namespace)
                e = ROSNodeEdge(pub_node_full_name,
                                node_full_name, topic_name,
                                topic_info.topic_type)
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

        if node_full_name not in node_to_lifecycle:
            node_to_lifecycle[node_full_name] = False

        if node_full_name not in node_to_component_manager:
            node_to_component_manager[node_full_name] = False

        servers = self._node.get_service_names_and_types_by_node(name, namespace)
        for service_name, service_types in servers:
            for service_type in service_types:
                edges.append(ROSNodeEdge(None, node_full_name, service_name, service_type))
                if 'get_state' in service_name and service_type == 'lifecycle_msgs/srv/GetState':
                    node_to_lifecycle[node_full_name] = True

                if 'list_nodes' in service_name and \
                   service_type == 'composition_interfaces/srv/ListNodes':
                    node_to_component_manager[node_full_name] = True

        return edges

    def get_action_edges(self, name, namespace):
        edges = []

        node_full_name = create_node_namespace(name, namespace)

        actions = rclpy.action.get_action_server_names_and_types_by_node(self._node,
                                                                         name,
                                                                         namespace)
        for action_name, action_types in actions:
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

                service_edges.extend(self.get_service_edges(name,
                                                            namespace,
                                                            node_to_lifecycle,
                                                            node_to_component_manager))

                action_edges.extend(self.get_action_edges(name, namespace))

            except rclpy.node.NodeNameNonExistentError:
                # It's possible that the node exited between when we saw it
                # in get_node_names_and_namespaces() and now.  If that happens,
                # just skip the processing.
                pass

        for name, namespace in self._node.get_node_names_and_namespaces():
            node_full_name = create_node_namespace(name, namespace)

            try:
                for service_name, service_types in \
                  self._node.get_client_names_and_types_by_node(name, namespace):
                    for service_type in service_types:
                        client_had_server = False
                        for e in service_edges:
                            if service_name == e.connection_name and \
                               service_type == e.connection_type:
                                e.from_node = node_full_name
                                client_had_server = True

                        if not client_had_server:
                            service_edges.append(ROSNodeEdge(node_full_name,
                                                             None,
                                                             service_name,
                                                             service_type))

                for action_name, action_types in \
                    rclpy.action.get_action_client_names_and_types_by_node(self._node,
                                                                           name,
                                                                           namespace):
                    for action_type in action_types:
                        action_client_had_server = False
                        for e in action_edges:
                            if action_name == e.connection_name and \
                               action_type == e.connection_type:
                                e.from_node = node_full_name
                                action_client_had_server = True

                        if not action_client_had_server:
                            action_edges.append(ROSNodeEdge(node_full_name,
                                                            None,
                                                            action_name,
                                                            action_type))

            except rclpy.node.NodeNameNonExistentError:
                # It's possible that the node exited between when we saw it
                # in get_node_names_and_namespaces() and now.  If that happens,
                # just skip the processing.
                pass

        return (topic_edges,
                service_edges,
                action_edges,
                node_to_lifecycle,
                node_to_component_manager)

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
            cm = ROSComponentManagerListNodesStateMachine(self._node, node_name)
            self._component_manager_nodes_clients[node_name] = cm

        return self._component_manager_nodes_clients[node_name].step()

    def shutdown(self):
        self._node.destroy_subscription(self._param_events_sub)
        # TODO(clalancette): We really should call destroy_node() here, but it
        # can race with calls to get_{topic,service}_edges()
        # self._node.destroy_node()
        self._executor.shutdown()
        self._spin_thread.join()
