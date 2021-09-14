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

import bisect
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
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
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


class ROSConnection:

    __slots__ = ('conn_name', 'conn_type', 'qos_profile')

    def __init__(self, conn_name, conn_type, qos_profile):
        self.conn_name = conn_name
        self.conn_type = conn_type
        self.qos_profile = qos_profile

    def __eq__(self, other):
        return self.conn_name == other.conn_name and \
            self.conn_type == other.conn_type and \
            self.qos_profile == other.qos_profile

    def __lt__(self, other):
        return self.conn_name < other.conn_name

    def __hash__(self):
        return hash(self.conn_name)


class ROSNode:

    __slots__ = ('name', 'is_lifecycle', 'is_component_manager', 'topic_publishers',
                 'service_clients', 'action_clients', 'topic_subscribers', 'service_servers',
                 'action_servers')

    def __init__(self, name):
        self.name = name
        self.is_lifecycle = False
        self.is_component_manager = False

        self.topic_publishers = []
        self.service_clients = []
        self.action_clients = []

        self.topic_subscribers = []
        self.service_servers = []
        self.action_servers = []

    def __eq__(self, other):
        return self.name == other.name and \
            self.topic_publishers == other.topic_publishers and \
            self.service_clients == other.service_clients and \
            self.action_clients == other.action_clients and \
            self.topic_subscribers == other.topic_subscribers and \
            self.service_servers == other.service_servers and \
            self.action_servers == other.action_servers

    def __lt__(self, other):
        return self.name < other.name

    def set_lifecycle(self, is_lifecycle):
        self.is_lifecycle = is_lifecycle

    def set_component_manager(self, is_component_manager):
        self.is_component_manager = is_component_manager

    def add_topic_publisher(self, topic_name, topic_type, qos_profile):
        bisect.insort_left(self.topic_publishers,
                           ROSConnection(topic_name, topic_type, qos_profile))

    def add_service_client(self, service_name, service_type):
        bisect.insort_left(self.service_clients,
                           ROSConnection(service_name, service_type, None))

    def add_action_client(self, action_name, action_type):
        bisect.insort_left(self.action_clients,
                           ROSConnection(action_name, action_type, None))

    def add_topic_subscriber(self, topic_name, topic_type, qos_profile):
        bisect.insort_left(self.topic_subscribers,
                           ROSConnection(topic_name, topic_type, qos_profile))

    def add_service_server(self, service_name, service_type):
        bisect.insort_left(self.service_servers,
                           ROSConnection(service_name, service_type, None))

    def add_action_server(self, action_name, action_type):
        bisect.insort_left(self.action_servers,
                           ROSConnection(action_name, action_type, None))


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
            # TODO(clalancette): If we never make it to HAS_INITIAL_RESPONSE,
            # or we are doing periodic refresh, then we are never destroying
            # this client.
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

    def reset(self):
        self._state = self.NEEDS_CLIENT


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
        if not self._list_state_machine.has_initial_response() or not list_param_response:
            return self._parameters

        self._param_names = list_param_response.result.names

        get_params_response = self._get_state_machine.step()
        if not self._get_state_machine.has_initial_response() or not get_params_response:
            return self._parameters

        # If the list of parameter names is a different length from the
        # get_parameters response, then probably the parameters changed between
        # the two calls.  Reset and try again.
        if len(self._param_names) != len(get_params_response.values):
            self._list_state_machine.reset()
            return self._parameters

        self._parameters_lock.acquire()
        for i, name in enumerate(self._param_names):
            if name not in self._parameters:
                val = get_ros_parameter_value(get_params_response.values[i])
                self._parameters[name] = val
            else:
                if self._parameters[name] is None:
                    del self._parameters[name]
        self._parameters_lock.release()

        return self._parameters

    def update_parameters_from_msg(self, msg):
        self._parameters_lock.acquire()
        for param in msg.new_parameters:
            self._parameters[param.name] = get_ros_parameter_value(param.value)
        for param in msg.changed_parameters:
            self._parameters[param.name] = get_ros_parameter_value(param.value)
        for param in msg.deleted_parameters:
            if param.name in self._parameters:
                del self._parameters[param.name]
            else:
                # If we get here and the parameter name is not in
                # self._parameters, then this callback may have happened before
                # we got the initial response from the 'get_parameters' service
                # call.  In that case, actually *add* the parameter to the dict
                # as "None", and then the 'get_parameters' machinery should
                # eventually remove it.
                self._parameters[param.name] = None
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

        self._param_state_machines = {}

        self._lc_state_state_machines = {}

        self._cm_nodes_state_machines = {}

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._shutting_down_lock = threading.Lock()
        self._spin_thread = threading.Thread(target=self._executor.spin)
        self._spin_thread.start()

    def get_nodes(self):
        nodes = []

        # If we ever see that the shutting_down_lock is locked, we know that
        # the program is going down.  Don't do any work in that case.
        if not self._shutting_down_lock.acquire(blocking=False):
            return nodes

        for name, namespace in self._node.get_node_names_and_namespaces():
            fully_qualified_name = create_node_namespace(name, namespace)
            rosnode = ROSNode(fully_qualified_name)

            try:
                # Topic publishers
                for topic_name, topic_types in \
                        self._node.get_publisher_names_and_types_by_node(name, namespace):

                    for info in self._node.get_publishers_info_by_topic(topic_name):
                        if info.endpoint_type != TopicEndpointTypeEnum.PUBLISHER:
                            # This should never happen, but just check it to be safe
                            continue

                        info_node_name = create_node_namespace(info.node_name, info.node_namespace)
                        if info_node_name != fully_qualified_name:
                            # TODO(clalancette): This wastes a lot of time
                            # iterating over things that don't pertain to us,
                            # can we do better?
                            continue

                        rosnode.add_topic_publisher(topic_name, info.topic_type, info.qos_profile)

                # Service clients
                for service_name, service_types in \
                        self._node.get_client_names_and_types_by_node(name, namespace):
                    for service_type in service_types:
                        rosnode.add_service_client(service_name, service_type)

                # Action clients
                action_clients = rclpy.action.get_action_client_names_and_types_by_node(
                    self._node, name, namespace)
                for action_name, action_types in action_clients:
                    for action_type in action_types:
                        rosnode.add_action_client(action_name, action_type)

                # Topic subscribers
                for topic_name, topic_types in \
                        self._node.get_subscriber_names_and_types_by_node(name, namespace):
                    for info in self._node.get_subscriptions_info_by_topic(topic_name):
                        if info.endpoint_type != TopicEndpointTypeEnum.SUBSCRIPTION:
                            # This should never happen, but just check it to be safe
                            continue

                        info_node_name = create_node_namespace(info.node_name, info.node_namespace)
                        if info_node_name != fully_qualified_name:
                            # TODO(clalancette): This wastes a lot of time
                            # iterating over things that don't pertain to us,
                            # can we do better?
                            continue

                        rosnode.add_topic_subscriber(topic_name, info.topic_type, info.qos_profile)

                # Service servers
                for service_name, service_types in \
                        self._node.get_service_names_and_types_by_node(name, namespace):
                    for service_type in service_types:
                        rosnode.add_service_server(service_name, service_type)
                        if 'get_state' in service_name and \
                           service_type == 'lifecycle_msgs/srv/GetState':
                            rosnode.set_lifecycle(True)

                        if 'list_nodes' in service_name and \
                           service_type == 'composition_interfaces/srv/ListNodes':
                            rosnode.set_component_manager(True)

                # Action servers
                action_servers = rclpy.action.get_action_server_names_and_types_by_node(
                    self._node, name, namespace)
                for action_name, action_types in action_servers:
                    for action_type in action_types:
                        rosnode.add_action_server(action_name, action_type)

            except rclpy.node.NodeNameNonExistentError:
                # It's possible that the node exited between when we saw it
                # in get_node_names_and_namespaces() and now.  If that happens,
                # just skip the processing.
                continue

            # TODO(clalancette): To be robust, we really should deal with other
            # errors from the rclpy API (like RCLError), not crash, and report
            # them somehow to the user

            bisect.insort_left(nodes, rosnode)

        self._shutting_down_lock.release()

        return nodes

    def parameter_event_cb(self, msg):
        # If we ever see that the shutting_down_lock is locked, we know that
        # the program is going down.  Don't do any work in that case.
        if not self._shutting_down_lock.acquire(blocking=False):
            return

        if msg.node in self._param_state_machines:
            self._param_state_machines[msg.node].update_parameters_from_msg(msg)

        self._shutting_down_lock.release()

    def get_node_parameters(self, node_name):
        # This method is useful to get all of the parameters and values from a
        # a node.  It is meant to be entirely non-blocking, which means that if
        # any part of the pipeline isn't ready, this will return an empty
        # dictionary.  Subsequent calls may then get the data.

        if not self._shutting_down_lock.acquire(blocking=False):
            return

        if node_name not in self._param_state_machines:
            self._param_state_machines[node_name] = ROSParameterStateMachine(self._node, node_name)

        ret = self._param_state_machines[node_name].step()

        self._shutting_down_lock.release()

        return ret

    def get_lifecycle_node_state(self, node_name):
        # This method is useful to get the lifecycle state of a Lifecycle node.
        # It should *only* be called on a Lifecycle node, but this method does
        # not enforce that.  Calling it on a non-Lifecycle node is not fatal,
        # but will eat up valuable resources in the network.  It is meant to be
        # entirely non-blocking, which means that if any part of the pipeline
        # isn't ready, this will return None.  Subsequent calls may get the data.

        if not self._shutting_down_lock.acquire(blocking=False):
            return

        if node_name not in self._lc_state_state_machines:
            self._lc_state_state_machines[node_name] = ROSLifecycleStateMachine(self._node,
                                                                                node_name)

        ret = self._lc_state_state_machines[node_name].step()

        self._shutting_down_lock.release()

        return ret

    def get_component_manager_nodes(self, node_name):
        # This method is useful to get the nodes that are part of a Component
        # Manager.  It should *only* be called on a Component Manager node, but
        # this method does not enforce that.  Calling it on a non-Component
        # Manager node is not fatal, but will eat up valuable resources in the
        # network.  It is meant to be entirely non-blocking, which means that
        # if any part of the pipeline isn't ready, this will return None.
        # Subsequent calls may get the data.

        if not self._shutting_down_lock.acquire(blocking=False):
            return

        if node_name not in self._cm_nodes_state_machines:
            cm = ROSComponentManagerListNodesStateMachine(self._node, node_name)
            self._cm_nodes_state_machines[node_name] = cm

        ret = self._cm_nodes_state_machines[node_name].step()

        self._shutting_down_lock.release()

        return ret

    def shutdown(self):
        self._shutting_down_lock.acquire()
        self._node.destroy_subscription(self._param_events_sub)
        self._node.destroy_node()
        self._executor.shutdown()
        self._spin_thread.join()
        # Note that we never release the lock; this is on purpose!  It's how
        # we signal to the other methods here that we are shutting down and they
        # should not attempt further work.
