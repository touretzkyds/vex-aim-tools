import QtQuick 6.5

Item {
    id: keymap

    focus: true

    property real panPixels: 50
    property real zoomFactor: 1.1

    signal viewPan(real deltaX, real deltaY)
    signal viewZoom(real factor)
    signal helpRequested()
    signal toggleRedisplay()
    signal requestRefresh()
    signal showObjects()
    signal showObstacles()
    signal showPose()
    signal centerView()
    signal exitRequested()

    function pan(dx, dy) {
        viewPan(dx * panPixels, dy * panPixels)
    }

    Keys.onPressed: (event) => {
        switch (event.key) {
        case Qt.Key_Left:
            pan(-1, 0);
            event.accepted = true;
            break;
        case Qt.Key_Right:
            pan(1, 0);
            event.accepted = true;
            break;
        case Qt.Key_Up:
            pan(0, 1);
            event.accepted = true;
            break;
        case Qt.Key_Down:
            pan(0, -1);
            event.accepted = true;
            break;
        case Qt.Key_Home:
            centerView();
            event.accepted = true;
            break;
        case Qt.Key_Greater:
            viewZoom(1 / zoomFactor);
            event.accepted = true;
            break;
        case Qt.Key_Less:
            viewZoom(zoomFactor);
            event.accepted = true;
            break;
        case Qt.Key_Comma:
            if (event.modifiers & Qt.ShiftModifier) {
                viewZoom(1 / zoomFactor);
                event.accepted = true;
            }
            break;
        case Qt.Key_Period:
            if (event.modifiers & Qt.ShiftModifier) {
                viewZoom(zoomFactor);
                event.accepted = true;
            }
            break;
        case Qt.Key_O:
            showObjects();
            event.accepted = true;
            break;
        case Qt.Key_B:
            showObstacles();
            event.accepted = true;
            break;
        case Qt.Key_P:
            showPose();
            event.accepted = true;
            break;
        case Qt.Key_Space:
            toggleRedisplay();
            event.accepted = true;
            break;
        case Qt.Key_H:
            helpRequested();
            event.accepted = true;
            break;
        case Qt.Key_Q:
        case Qt.Key_Escape:
            exitRequested();
            event.accepted = true;
            break;
        case Qt.Key_F5:
            requestRefresh();
            event.accepted = true;
            break;
        case Qt.Key_Plus:
        case Qt.Key_Equal:
            viewZoom(1 / zoomFactor);
            event.accepted = true;
            break;
        case Qt.Key_Minus:
        case Qt.Key_Underscore:
            viewZoom(zoomFactor);
            event.accepted = true;
            break;
        default:
            break;
        }
    }
}
