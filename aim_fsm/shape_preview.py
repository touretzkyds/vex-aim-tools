"""Isolated Qt renderer for reviewing generated display shapes; no robot connection."""
import json
import sys
import tempfile
from pathlib import Path


def render(parts, output):
    from PyQt6.QtGui import QGuiApplication
    from PyQt6.QtQuick import QQuickView
    from PyQt6.QtCore import QUrl, QTimer

    app = QGuiApplication([])
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'preview.qml'
        path.write_text(make_qml(parts), encoding='utf-8')
        view = QQuickView()
        view.setSource(QUrl.fromLocalFile(str(path)))
        if view.errors():
            raise ValueError([e.toString() for e in view.errors()])
        # Qt's offscreen platform can return a blank Quick3D frame on Windows.
        view.setPosition(-10000, -10000)
        view.show()

        def save():
            if not view.grabWindow().save(str(output)):
                app.exit(1)
            else:
                app.quit()
        QTimer.singleShot(1200, save)
        return app.exec()


def make_qml(parts):
    nodes = []
    for part in parts:
        def vector(key, divisor=1):
            return ','.join(str(v / divisor) for v in part[key])
        nodes.append('''        Model {
            source: "#%s"
            position: Qt.vector3d(%s)
            scale: Qt.vector3d(%s)
            eulerRotation: Qt.vector3d(%s)
            materials: PrincipledMaterial { baseColor: "%s"; roughness: 0.75 }
        }''' % (part['shape'].title(), vector('position'), vector('size', 100),
                vector('rotation'), part['color']))
    return '''import QtQuick
import QtQuick3D
Rectangle {
    width: 800; height: 600; color: "#e5e7eb"
    View3D {
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#e5e7eb"
            antialiasingMode: SceneEnvironment.MSAA
        }
        Node {
            position.y: 50
            eulerRotation: Qt.vector3d(-15, 30, 0)
            PerspectiveCamera { id: camera; z: 180; clipNear: 0.1; clipFar: 2000 }
        }
        camera: camera
        DirectionalLight {
            eulerRotation: Qt.vector3d(-30, -30, 0)
            brightness: 1.2
            ambientColor: "#777777"
        }
''' + '\n'.join(nodes) + '''
    }
}
'''


if __name__ == '__main__':
    # Load the sibling module without initializing aim_fsm or its robot dependencies.
    from openvocab_shape import validate

    parts = validate(json.loads(Path(sys.argv[1]).read_text(encoding='utf-8')))
    sys.exit(render(parts, Path(sys.argv[2])))
