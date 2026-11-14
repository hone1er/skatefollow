"""GoPro over USB (HERO 8/9/10 and newer): find it, put it in webcam mode, build the stream URL.

How a GoPro looks to Linux when plugged into the Pi over USB-C (camera set to
Preferences > Connections > USB Connection > "GoPro Connect", not MTP):
  * it enumerates as a USB network adapter (RNDIS/NCM -> interface usb0 or enx...)
  * it runs a DHCP server on that link and hands the Pi 172.2X.1YZ.50 (or .54, any lease)
  * the camera itself is 172.2X.1YZ.51 (X, Y, Z come from the camera's serial number)
  * HTTP GET requests on the camera control it; webcam mode then pushes an MPEG-TS
    stream over UDP to the host on port 8554 (H.264, 1080p/720p/480p, 30 fps).
  * In webcam mode the camera does NOT record to its SD card. Recording of the
    1080p stream on the Pi is done by camera_node (record:=true), zero CPU cost.

Two HTTP APIs exist; we try the newer one and fall back:
  Open GoPro (HERO10+, also HERO9 with recent firmware), port 8080:
      /gopro/webcam/start?res=<4|7|12>&fov=<f>&port=8554    4 = 480p, 7 = 720p, 12 = 1080p
      /gopro/webcam/stop  /gopro/webcam/exit  /gopro/webcam/status
  Legacy webcam API (HERO8/9), port 80:
      /gp/gpWebcam/SETTINGS?fov=<f>   then   /gp/gpWebcam/START?res=<1080|720|480>
      /gp/gpWebcam/STOP
  fov codes (both APIs): 0 wide, 2 narrow, 3 superview, 4 linear.

Everything that touches the network lives in GoProControl / find_gopro; the
parsing helpers above them are pure and unit-tested.

CLI for bench use:  ros2 run honebot_camera gopro_ctl {find|start|stop|status}
"""

import ipaddress
import re
import subprocess
import sys
import urllib.error
import urllib.request

GOPRO_UDP_PORT = 8554
RES_TO_OPENGOPRO = {480: 4, 720: 7, 1080: 12}
PRIVATE_172 = ipaddress.IPv4Network('172.16.0.0/12')

_IP_ADDR_RE = re.compile(r'^\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)', re.M)


def gopro_ip_from_host_ip(host_ip):
    """The camera is always .51 on its 172.2X.1YZ.0/24 USB link; the host gets .50 or another DHCP
    lease (a HERO9 handed out .54). None if host_ip is not on such a link."""
    try:
        ip = ipaddress.IPv4Address(host_ip)
    except ipaddress.AddressValueError:
        return None
    if ip not in PRIVATE_172:
        return None
    octets = str(ip).split('.')
    if octets[3] == '51':
        return None
    return '.'.join(octets[:3] + ['51'])


def parse_ip_addr(text):
    """Parse `ip -4 -o addr` output into [(interface, ipv4), ...]."""
    return [(m.group(1), m.group(2)) for m in _IP_ADDR_RE.finditer(text)]


def find_gopro_in(addresses):
    """From [(iface, ip)] return (iface, host_ip, gopro_ip) of the first GoPro link, or None."""
    for iface, ip in addresses:
        cam = gopro_ip_from_host_ip(ip)
        if cam is not None:
            return iface, ip, cam
    return None


def find_gopro():
    """Look at the Pi's interfaces for a GoPro USB link. Returns (iface, host_ip, gopro_ip) or None."""
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True, text=True, timeout=2.0,
                             check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    return find_gopro_in(parse_ip_addr(out))


def stream_url(host='0.0.0.0', port=GOPRO_UDP_PORT):
    """URL OpenCV/FFmpeg opens to receive the camera's MPEG-TS over UDP.

    fifo_size is in 188-byte packets (~9 MB); overrun_nonfatal keeps a burst from killing the capture.
    """
    return f'udp://{host}:{port}?overrun_nonfatal=1&fifo_size=50000'


def opengopro_start_path(res=1080, fov=4, port=GOPRO_UDP_PORT):
    code = RES_TO_OPENGOPRO.get(int(res))
    if code is None:
        raise ValueError(f'res must be one of {sorted(RES_TO_OPENGOPRO)}, not {res}')
    # HERO9 (HD9.01.01.72) answers HTTP 500 if a `port` parameter is present, so it is only sent
    # when a non-default port is requested (newer cameras accept it).
    path = f'/gopro/webcam/start?res={code}&fov={int(fov)}'
    if int(port) != GOPRO_UDP_PORT:
        path += f'&port={int(port)}'
    return path


def legacy_start_paths(res=1080, fov=4):
    if int(res) not in RES_TO_OPENGOPRO:
        raise ValueError(f'res must be one of {sorted(RES_TO_OPENGOPRO)}, not {res}')
    return [f'/gp/gpWebcam/SETTINGS?fov={int(fov)}', f'/gp/gpWebcam/START?res={int(res)}']


class GoProControl:
    """HTTP control of one camera. All methods return a short human-readable string or raise OSError."""

    def __init__(self, ip, timeout=3.0):
        self.ip = ip
        self.timeout = timeout
        self.api = None            # 'opengopro' or 'legacy' once something worked

    def _get(self, path, port=80):
        url = f'http://{self.ip}:{port}{path}'
        req = urllib.request.Request(url, headers={'Connection': 'close'})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return resp.status, resp.read(2048).decode('utf-8', errors='replace')

    def _try(self, path, port=80):
        try:
            status, body = self._get(path, port)
            return status == 200, body
        except (urllib.error.URLError, OSError, ValueError) as e:
            return False, str(e)

    def start_webcam(self, res=1080, fov=4, port=GOPRO_UDP_PORT):
        # Open GoPro first (HERO10+; HERO9 on new firmware). Enabling wired control is harmless if unsupported.
        self._try('/gopro/camera/control/wired_usb?p=1', 8080)
        ok, body = self._try(opengopro_start_path(res, fov, port), 8080)
        if ok:
            self.api = 'opengopro'
            return f'webcam started via Open GoPro ({res}p, fov {fov}): {body.strip()[:80]}'
        err1 = body
        # Legacy API (HERO8/9). The stream always goes to UDP 8554 on the host.
        for path in legacy_start_paths(res, fov):
            ok, body = self._try(path, 80)
            if not ok:
                raise OSError(f'GoPro {self.ip} did not accept {path!r}: {body.strip()[:80]} '
                              f'(Open GoPro attempt said: {err1.strip()[:80]})')
        self.api = 'legacy'
        return f'webcam started via legacy API ({res}p, fov {fov})'

    def stop_webcam(self):
        if self.api == 'opengopro':
            ok, body = self._try('/gopro/webcam/stop', 8080)
            self._try('/gopro/webcam/exit', 8080)
            return 'webcam stopped' if ok else f'stop failed: {body}'
        ok, body = self._try('/gp/gpWebcam/STOP', 80)
        if not ok and self.api is None:
            ok, body = self._try('/gopro/webcam/stop', 8080)
        return 'webcam stopped' if ok else f'stop failed: {body}'

    def status(self):
        ok, body = self._try('/gopro/webcam/status', 8080)
        if ok:
            return f'Open GoPro status: {body.strip()}'
        ok, body = self._try('/gp/gpWebcam/STATUS', 80)
        if ok:
            return f'legacy status: {body.strip()}'
        return f'no answer from {self.ip}: {body}'


def main(argv=None):
    """Bench CLI: ros2 run honebot_camera gopro_ctl find|start|stop|status [res] [fov]."""
    argv = sys.argv[1:] if argv is None else argv
    cmd = argv[0] if argv else 'find'
    found = find_gopro()
    if found is None:
        print('No GoPro USB link found: no interface with a 172.2X.1YZ.50 address.\n'
              'Check: cable plugged in, camera on, USB Connection = GoPro Connect, and the netplan file\n'
              'from honebot_camera/netplan installed so the usb0/enx* interface gets DHCP.')
        return 1
    iface, host_ip, cam_ip = found
    print(f'GoPro link: {iface} host {host_ip} camera {cam_ip}')
    if cmd == 'find':
        return 0
    ctl = GoProControl(cam_ip)
    try:
        if cmd == 'start':
            res = int(argv[1]) if len(argv) > 1 else 1080
            fov = int(argv[2]) if len(argv) > 2 else 4
            print(ctl.start_webcam(res, fov))
            print(f'stream: {stream_url()}   (e.g. on the Mac: ffplay -fflags nobuffer {stream_url()})')
        elif cmd == 'stop':
            print(ctl.stop_webcam())
        elif cmd == 'status':
            print(ctl.status())
        else:
            print(f'unknown command {cmd!r}; use find|start|stop|status')
            return 2
    except OSError as e:
        print(f'error: {e}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
