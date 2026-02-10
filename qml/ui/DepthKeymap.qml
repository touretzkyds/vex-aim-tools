import QtQuick 6.5

Item {
    id: keymap

    focus: true

    signal helpRequested()
    signal toggleLiveRefresh()
    signal toggleGradientSourceRequested()
    signal requestSingleRefresh()
    signal exitRequested()

    Keys.onPressed: (event) => {
        switch (event.key) {
        case Qt.Key_H:
            helpRequested()
            event.accepted = true
            break
        case Qt.Key_Space:
            toggleLiveRefresh()
            event.accepted = true
            break
        case Qt.Key_G:
            toggleGradientSourceRequested()
            event.accepted = true
            break
        case Qt.Key_R:
            requestSingleRefresh()
            event.accepted = true
            break
        case Qt.Key_Q:
        case Qt.Key_Escape:
            exitRequested()
            event.accepted = true
            break
        default:
            break
        }
    }
}
