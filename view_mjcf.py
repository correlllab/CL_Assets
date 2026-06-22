import argparse
import time

import mujoco
import mujoco.viewer


def view_mjcf(filename):
    model = mujoco.MjModel.from_xml_path(filename)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize MJCF/XML model using MuJoCo viewer')
    parser.add_argument('filename', type=str, help='MJCF/XML filename')
    args = parser.parse_args()
    view_mjcf(args.filename)
