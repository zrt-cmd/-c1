"""Convert the robot URDF to Gazebo SDF with body-fixed mecanum friction."""

import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def build_mecanum_sdf(urdf_file):
    converted = subprocess.run(
        ['gz', 'sdf', '-p', urdf_file], check=True, capture_output=True, text=True
    )
    root = ET.fromstring(converted.stdout)
    model = root.find('model')
    if model is None:
        raise ValueError('Gazebo conversion produced no robot model')

    ET.register_namespace('gz', 'http://gazebosim.org/schema')
    for wheel in ('wheel_lf_link', 'wheel_rf_link', 'wheel_lb_link', 'wheel_rb_link'):
        direction = model.find(
            f"link[@name='{wheel}']/collision/surface/friction/ode/fdir1"
        )
        if direction is None:
            raise ValueError(f'Missing mecanum friction direction for {wheel}')
        direction.set('{http://gazebosim.org/schema}expressed_in', 'base_footprint')

    drive = model.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    if drive is None:
        raise ValueError('MecanumDrive plugin is missing from the converted model')

    fd, sdf_file = tempfile.mkstemp(prefix='car1_mecanum_', suffix='.sdf')
    with os.fdopen(fd, 'wb') as output:
        ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return sdf_file
