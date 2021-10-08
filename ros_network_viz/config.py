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

from dataclasses import dataclass


@dataclass
class Configuration:
    list_hidden_nodes: bool
    list_hidden_services: bool
    list_hidden_topics: bool
    list_parameter_services: bool
    list_lifecycle_services: bool


config = Configuration(
    list_hidden_nodes=False,
    list_hidden_services=False,
    list_hidden_topics=False,
    list_parameter_services=False,
    list_lifecycle_services=False
)
