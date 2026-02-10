import QtQuick 6.5

Item {
    id: keymap

    focus: true

    property real translateStepSmall: 10
    property real translateStepLarge: 40
    property real rotateStepSmall: 22.5
    property real rotateStepLarge: 90
    property real panPixels: 50
    property real zoomFactor: 1.1

    signal driveForward(real distance)
    signal strafe(real distance)
    signal turn(real angleDeg)

    signal evaluateParticles()
    signal resampleParticles()
    signal updateOccupancyGridRequested()
    signal resetParticles()
    signal jitterParticles()
    signal clearLandmarks()
    signal showLandmarks()
    signal showObjects()
    signal showPose()
    signal showBestParticle()
    signal reportVariance()
    signal toggleVerbose()

    signal helpRequested()
    signal autoCenterToggled()
    signal exitRequested()
    signal toggleRedisplay()
    signal centerView()

    signal viewPan(real deltaX, real deltaY)
    signal viewZoom(real factor)

    function pan(forward, strafe) {
        viewPan(strafe * panPixels, forward * panPixels)
    }

    Keys.onPressed: (event) => {
        const shift = event.modifiers & Qt.ShiftModifier
        switch (event.key) {
        case Qt.Key_W:
            driveForward(shift ? translateStepLarge : translateStepSmall)
            event.accepted = true
            break
        case Qt.Key_S:
            driveForward(-(shift ? translateStepLarge : translateStepSmall))
            event.accepted = true
            break
        case Qt.Key_J:
            strafe(shift ? translateStepLarge : translateStepSmall)
            event.accepted = true
            break
        case Qt.Key_K:
            strafe(-(shift ? translateStepLarge : translateStepSmall))
            event.accepted = true
            break
        case Qt.Key_A:
            turn(shift ? rotateStepLarge : rotateStepSmall)
            event.accepted = true
            break
        case Qt.Key_D:
            turn(-(shift ? rotateStepLarge : rotateStepSmall))
            event.accepted = true
            break
        case Qt.Key_E:
            evaluateParticles()
            event.accepted = true
            break
        case Qt.Key_R:
            resampleParticles()
            event.accepted = true
            break
        case Qt.Key_M:
            updateOccupancyGridRequested()
            event.accepted = true
            break
        case Qt.Key_Z:
            if (shift) {
                jitterParticles()
            } else {
                resetParticles()
            }
            event.accepted = true
            break
        case Qt.Key_C:
            clearLandmarks()
            event.accepted = true
            break
        case Qt.Key_L:
            showLandmarks()
            event.accepted = true
            break
        case Qt.Key_O:
            showObjects()
            event.accepted = true
            break
        case Qt.Key_P:
            if (shift) {
                showBestParticle()
            } else {
                showPose()
            }
            event.accepted = true
            break
        case Qt.Key_V:
            if (shift) {
                reportVariance()
            } else {
                toggleVerbose()
            }
            event.accepted = true
            break
        case Qt.Key_H:
            helpRequested()
            event.accepted = true
            break
        case Qt.Key_Home:
            centerView()
            event.accepted = true
            break
        case Qt.Key_Space:
            autoCenterToggled()
            event.accepted = true
            break
        case Qt.Key_Left:
            pan(0, -1)
            event.accepted = true
            break
        case Qt.Key_Right:
            pan(0, 1)
            event.accepted = true
            break
        case Qt.Key_Up:
            pan(1, 0)
            event.accepted = true
            break
        case Qt.Key_Down:
            pan(-1, 0)
            event.accepted = true
            break
        case Qt.Key_Plus:
        case Qt.Key_Equal:
            viewZoom(1 / zoomFactor)
            event.accepted = true
            break
        case Qt.Key_Minus:
        case Qt.Key_Underscore:
            viewZoom(zoomFactor)
            event.accepted = true
            break
        case Qt.Key_Comma:
            if (shift) {
                viewZoom(1 / zoomFactor)
                event.accepted = true
            }
            break
        case Qt.Key_Period:
            if (shift) {
                viewZoom(zoomFactor)
                event.accepted = true
            }
            break
        case Qt.Key_Dollar:
            toggleRedisplay()
            event.accepted = true
            break
        case Qt.Key_Q:
            exitRequested()
            event.accepted = true
            break
        case Qt.Key_Escape:
            exitRequested()
            event.accepted = true
            break
        default:
            break
        }
    }
}
