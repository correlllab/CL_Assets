import os
import argparse
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


def _attach_frame_shape(handle):
    try:
        import meshcat_shapes
    except ImportError as exc:
        raise RuntimeError("--frames requires the meshcat-shapes package") from exc

    meshcat_shapes.frame(
        handle,
        axis_length=0.08,
        axis_thickness=0.003,
        opacity=0.9,
        origin_radius=0.008,
    )


def _resolve_frames(model, frame_names):
    resolved = []
    for name in frame_names:
        try:
            frame_id = model.getFrameId(name)
        except Exception:
            print(f"Skipping unknown frame: {name}")
            continue

        if frame_id < 0 or frame_id >= len(model.frames):
            print(f"Skipping unknown frame: {name}")
            continue

        resolved.append((name, frame_id))
    return resolved


def view_urdf(filename, frame_names=None):
    # package dir as parent folder
    model_path = os.path.dirname(os.path.abspath(filename))

    # load model
    model, visual, collision = pin.buildModelsFromUrdf(
        filename,
        package_dirs=model_path
    )
    viz = MeshcatVisualizer(model, visual, collision)
    viz.initViewer()
    viz.loadViewerModel()

    frame_nodes = []
    if frame_names:
        data = model.createData()
        for name, frame_id in _resolve_frames(model, frame_names):
            node = viz.viewer[f"frames/{name}"]
            _attach_frame_shape(node)
            frame_nodes.append((node, frame_id))

    while True:
        q = pin.neutral(model)
        # display robot at neutral configuration
        viz.display(q)

        if frame_nodes:
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            for node, frame_id in frame_nodes:
                node.set_transform(data.oMf[frame_id].homogeneous)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize URDF model using Meshcat')
    parser.add_argument('filename', type=str, help='URDF filename')
    parser.add_argument('--frames', nargs='+', default=[], help='Frame names to show in Meshcat')
    args = parser.parse_args()
    view_urdf(args.filename, args.frames)
