import pytest

from honebot_camera.gopro import (find_gopro_in, gopro_ip_from_host_ip, legacy_start_paths,
                                  opengopro_start_path, parse_ip_addr, stream_url)

IP_ADDR_OUT = """\
1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever
3: wlan0    inet 192.168.10.122/24 metric 600 brd 192.168.10.255 scope global dynamic wlan0\\       valid_lft 80000sec preferred_lft 80000sec
7: enx0a1b2c3d4e5f    inet 172.25.123.50/24 brd 172.25.123.255 scope global dynamic enx0a1b2c3d4e5f\\       valid_lft 3000sec preferred_lft 3000sec
"""


def test_camera_ip_is_dot_51_of_host_link():
    assert gopro_ip_from_host_ip('172.25.123.50') == '172.25.123.51'
    assert gopro_ip_from_host_ip('172.22.120.54') == '172.22.120.51'      # what a HERO9 actually handed out


def test_other_addresses_are_not_a_gopro():
    assert gopro_ip_from_host_ip('192.168.10.122') is None     # home wifi
    assert gopro_ip_from_host_ip('172.25.123.51') is None      # that is the camera itself
    assert gopro_ip_from_host_ip('10.0.0.50') is None
    assert gopro_ip_from_host_ip('garbage') is None


def test_parse_ip_addr_output():
    addrs = parse_ip_addr(IP_ADDR_OUT)
    assert ('wlan0', '192.168.10.122') in addrs
    assert ('enx0a1b2c3d4e5f', '172.25.123.50') in addrs
    assert len(addrs) == 3


def test_find_gopro_in_interfaces():
    assert find_gopro_in(parse_ip_addr(IP_ADDR_OUT)) == ('enx0a1b2c3d4e5f', '172.25.123.50', '172.25.123.51')
    assert find_gopro_in([('wlan0', '192.168.10.122')]) is None
    assert find_gopro_in([]) is None


def test_stream_url_and_api_paths():
    assert stream_url().startswith('udp://0.0.0.0:8554?')
    assert stream_url('127.0.0.1', 8555).startswith('udp://127.0.0.1:8555?')
    assert opengopro_start_path(1080, 4, 8554) == '/gopro/webcam/start?res=12&fov=4'
    assert opengopro_start_path(1080, 4, 9000) == '/gopro/webcam/start?res=12&fov=4&port=9000'
    assert opengopro_start_path(720, 0) == '/gopro/webcam/start?res=7&fov=0'
    assert legacy_start_paths(1080, 4) == ['/gp/gpWebcam/SETTINGS?fov=4', '/gp/gpWebcam/START?res=1080']
    with pytest.raises(ValueError):
        opengopro_start_path(900)
    with pytest.raises(ValueError):
        legacy_start_paths(4000)
