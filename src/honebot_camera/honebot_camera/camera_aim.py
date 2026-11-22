"""Camera aim node: keep the skater centred by panning the camera servo.

Subscribes
    camera/target     geometry_msgs/Vector3Stamped   x = skater bearing from the camera axis (camera_node)
    target            geometry_msgs/PointStamped     UWB target in base_link (optional fallback when the
                                                     detector loses the skater)
Publishes at rate_hz
    camera/pan_cmd    std_msgs/Float32   -1..1 servo command, forwarded by esp32_bridge as `P,<pan>`
    camera/pan_angle  std_msgs/Float32   the commanded pan (rad, robot frame, +left); camera_node uses it
    camera/bearing    std_msgs/Float32   pan + camera bearing = where the skater is in the robot frame
                                         (rad). Only while the detector sees them. A follower could use it
                                         when UWB is missing.

The servo is open loop; see aim.py. Parameters are live-settable (ros2 param set /camera_aim kp 0.4).
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32

from honebot_camera.aim import AimGains, PanController


class CameraAim(Node):
    def __init__(self):
        super().__init__('camera_aim')
        p = self.declare_parameter
        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.pan_sign = float(p('pan_sign', 1.0).value)        # -1 if the servo turns the wrong way
        self.use_uwb = p('use_uwb_fallback', True).value
        self.uwb_timeout = float(p('uwb_timeout', 0.5).value)
        self.meas_max_age = float(p('measurement_max_age', 0.3).value)   # ignore older detections
        self.gains = AimGains(
            kp=float(p('kp', 0.6).value),
            deadband=math.radians(float(p('deadband_deg', 1.0).value)),
            max_rate=math.radians(float(p('max_rate_deg_s', 150.0).value)),
            half_range=math.radians(float(p('half_range_deg', 70.0).value)),   # measured angle at pan +-1
            lost_timeout=float(p('lost_timeout', 0.5).value),
            recenter_timeout=float(p('recenter_timeout', 3.0).value),
            recenter_rate=math.radians(float(p('recenter_rate_deg_s', 30.0).value)),
        )
        self.ctl = PanController(self.gains)
        self.add_on_set_parameters_callback(self._on_params)

        self.pan_pub = self.create_publisher(Float32, 'camera/pan_cmd', 10)
        self.angle_pub = self.create_publisher(Float32, 'camera/pan_angle', 10)
        self.bearing_pub = self.create_publisher(Float32, 'camera/bearing', 10)
        self.create_subscription(Vector3Stamped, 'camera/target', self._on_target, 10)
        self.create_subscription(PointStamped, 'target', self._on_uwb, 10)

        self.meas = None            # (bearing_err, time) of the newest unconsumed detection
        self.uwb_bearing = None
        self.uwb_time = None
        self.was_tracking = False
        self.last_time = self._now()
        self.create_timer(1.0 / self.rate_hz, self._step)
        self.get_logger().info(f'camera_aim up: kp {self.gains.kp}, half range '
                               f'{math.degrees(self.gains.half_range):.0f} deg, rate {self.rate_hz} Hz')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_params(self, params):
        updates = []
        for prm in params:
            if prm.name == 'half_range_deg':
                val = float(prm.value)
                if not math.isfinite(val) or val <= 0.0:
                    return SetParametersResult(successful=False, reason='half_range_deg must be finite and > 0')
                updates.append(('half_range', math.radians(val)))
            elif prm.name in ('kp', 'lost_timeout', 'recenter_timeout', 'uwb_timeout', 'measurement_max_age'):
                val = float(prm.value)
                if not math.isfinite(val) or val < 0.0:
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be finite and >= 0')
                updates.append((prm.name, val))
            elif prm.name in ('deadband_deg', 'max_rate_deg_s', 'recenter_rate_deg_s'):
                val = float(prm.value)
                if not math.isfinite(val) or val < 0.0:
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be finite and >= 0')
                updates.append((prm.name, math.radians(val)))
            elif prm.name == 'pan_sign':
                val = float(prm.value)
                if not math.isfinite(val) or val == 0.0:
                    return SetParametersResult(successful=False, reason='pan_sign must be finite and non-zero')
                updates.append(('pan_sign', val))
            elif prm.name == 'use_uwb_fallback':
                updates.append(('use_uwb', bool(prm.value)))
        for name, val in updates:
            if name == 'half_range':
                self.gains.half_range = val
            elif name == 'deadband_deg':
                self.gains.deadband = val
            elif name == 'max_rate_deg_s':
                self.gains.max_rate = val
            elif name == 'recenter_rate_deg_s':
                self.gains.recenter_rate = val
            elif name in ('kp', 'lost_timeout', 'recenter_timeout'):
                setattr(self.gains, name, val)
            elif name == 'uwb_timeout':
                self.uwb_timeout = val
            elif name == 'measurement_max_age':
                self.meas_max_age = val
            elif name == 'pan_sign':
                self.pan_sign = val
            elif name == 'use_uwb':
                self.use_uwb = val
        return SetParametersResult(successful=True)

    def _on_target(self, msg: Vector3Stamped):
        b = float(msg.vector.x)
        if not math.isfinite(b):
            return
        self.meas = (b, self._now())
        # Fresh robot-frame bearing of the skater: commanded pan + where they sit in the image.
        self.bearing_pub.publish(Float32(data=float(self.ctl.pan + b)))

    def _on_uwb(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self.uwb_bearing = math.atan2(y, x)
        self.uwb_time = self._now()

    def _step(self):
        now = self._now()
        dt = now - self.last_time
        self.last_time = now
        err = None
        if self.meas is not None:
            b, t = self.meas
            self.meas = None
            if now - t <= self.meas_max_age:
                err = b
        fallback = None
        if self.use_uwb and self.uwb_time is not None and now - self.uwb_time <= self.uwb_timeout:
            fallback = self.uwb_bearing
        pan = self.ctl.update(now, dt, err, fallback)
        self.pan_pub.publish(Float32(data=float(self.ctl.normalized(self.pan_sign))))
        self.angle_pub.publish(Float32(data=float(pan)))
        tracking = self.ctl.tracking(now)
        if tracking != self.was_tracking:
            self.get_logger().info('tracking skater' if tracking else
                                   ('skater lost; following UWB' if fallback is not None else
                                    'skater lost; will recentre'))
            self.was_tracking = tracking


def main(args=None):
    rclpy.init(args=args)
    node = CameraAim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
