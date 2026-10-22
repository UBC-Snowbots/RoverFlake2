from unittest.mock import Mock, patch

import pytest
from rclpy.node import Node
from std_msgs.msg import Empty

from ptz_cam.ip_camera_zoom import IPCameraZoomNode


ACTIONS = [
    ('zoom_in_start', 'zoomin'),
    ('zoom_out_start', 'zoomout'),
    ('zoom_stop', 'stop'),
    ('focus_in_start', 'focusin'),
    ('focus_out_start', 'focusout'),
    ('focus_stop', 'stop'),
]


@pytest.fixture
def zoom_node():
    values = {
        'camera_ip': '192.168.1.88',
        'http_port': 80,
        'zoom_endpoint_path': '/web/cgi-bin/hi3510/ptzctrl.cgi',
        'camera_user': 'admin',
        'camera_pass': 'admin',
        'zoom_speed': 45,
        'zoom_step': 0,
        'request_timeout': 2.0,
    }
    with (
        patch.object(Node, '__init__', return_value=None),
        patch.object(Node, 'declare_parameter'),
        patch.object(Node, 'get_parameter', side_effect=lambda name: Mock(value=values[name])),
        patch.object(Node, 'get_logger', return_value=Mock()),
        patch.object(Node, 'create_subscription') as subscribe,
        patch.object(Node, 'create_service') as service,
    ):
        yield IPCameraZoomNode(), subscribe, service


def test_registers_command_topics_instead_of_services(zoom_node):
    _, subscribe, service = zoom_node
    assert subscribe.call_count == len(ACTIONS)
    assert [call.args[1] for call in subscribe.call_args_list] == [
        name for name, _ in ACTIONS
    ]
    for call in subscribe.call_args_list:
        assert call.args[0] is Empty
        assert call.args[3] == 10
    service.assert_not_called()


@pytest.mark.parametrize('index,action', enumerate(action for _, action in ACTIONS))
def test_topic_message_sends_expected_http_action(zoom_node, index, action):
    node, subscribe, _ = zoom_node
    callback = subscribe.call_args_list[index].args[2]
    with patch('ptz_cam.ip_camera_zoom.requests.get', return_value=Mock(status_code=200)) as get:
        callback(Empty())
    get.assert_called_once_with(
        'http://192.168.1.88:80/web/cgi-bin/hi3510/ptzctrl.cgi',
        params={'-step': 0, '-act': action, '-speed': 45},
        auth=node._auth,
        timeout=2.0,
    )


def test_topic_http_failure_is_logged(zoom_node):
    node, subscribe, _ = zoom_node
    callback = subscribe.call_args_list[0].args[2]
    with patch('ptz_cam.ip_camera_zoom.requests.get', return_value=Mock(status_code=401)):
        callback(Empty())
    node.get_logger().error.assert_called_once_with(
        "'zoomin' unauthorized (401): check camera_user/camera_pass"
    )
