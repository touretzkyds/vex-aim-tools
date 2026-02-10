import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "layers"
import "ui" as UI

FocusScope {
    id: root
    objectName: "particleViewRoot"
    focus: true

    property alias particleCanvas: canvas
    property var particleModelRef: particleModel
    property var landmarkModelRef: landmarkModel
    property var particleSummaryRef: particleSummary
    property var viewStateRef: viewState

    implicitWidth: 640
    implicitHeight: 640

    Rectangle {
        anchors.fill: parent
        color: "#0b1119"
    }

    ParticleCanvas {
        id: canvas
        objectName: "particleCanvas"
        anchors.fill: parent
        particleModel: root.particleModelRef
        landmarkModel: root.landmarkModelRef
        particleSummary: root.particleSummaryRef
        viewState: root.viewStateRef
    }

    UI.ParticleKeymap {
        id: keymap
        anchors.fill: parent
        focus: true
        onViewPan: function(deltaX, deltaY) { adjustCenter(deltaX, deltaY) }
        onViewZoom: function(factor) { adjustZoom(factor) }
        onAutoCenterToggled: {
            if (viewerApp && typeof viewerApp.toggleAutoCenter === "function")
                viewerApp.toggleAutoCenter()
        }
        onHelpRequested: {
            if (viewerApp && typeof viewerApp.printHelp === "function")
                viewerApp.printHelp()
        }
        onExitRequested: {
            if (viewerApp && typeof viewerApp.requestQuit === "function")
                viewerApp.requestQuit()
            else
                Qt.quit()
        }
        onDriveForward: function(distance) {
            if (viewerApp && typeof viewerApp.driveForward === "function")
                viewerApp.driveForward(distance)
        }
        onStrafe: function(distance) {
            if (viewerApp && typeof viewerApp.strafe === "function")
                viewerApp.strafe(distance)
        }
        onTurn: function(angleDeg) {
            if (viewerApp && typeof viewerApp.turnDegrees === "function")
                viewerApp.turnDegrees(angleDeg)
        }
        onEvaluateParticles: {
            if (viewerApp && typeof viewerApp.evaluateParticles === "function")
                viewerApp.evaluateParticles()
        }
        onResampleParticles: {
            if (viewerApp && typeof viewerApp.resampleParticles === "function")
                viewerApp.resampleParticles()
        }
        onUpdateOccupancyGridRequested: {
            if (viewerApp && typeof viewerApp.updateOccupancyGridFromCurrentFrame === "function")
                viewerApp.updateOccupancyGridFromCurrentFrame()
        }
        onResetParticles: {
            if (viewerApp && typeof viewerApp.resetParticles === "function")
                viewerApp.resetParticles()
        }
        onJitterParticles: {
            if (viewerApp && typeof viewerApp.jitterParticles === "function")
                viewerApp.jitterParticles()
        }
        onClearLandmarks: {
            if (viewerApp && typeof viewerApp.clearLandmarks === "function")
                viewerApp.clearLandmarks()
        }
        onShowLandmarks: {
            if (viewerApp && typeof viewerApp.showLandmarks === "function")
                viewerApp.showLandmarks()
        }
        onShowObjects: {
            if (viewerApp && typeof viewerApp.showObjects === "function")
                viewerApp.showObjects()
        }
        onShowPose: {
            if (viewerApp && typeof viewerApp.showPose === "function")
                viewerApp.showPose()
        }
        onShowBestParticle: {
            if (viewerApp && typeof viewerApp.showBestParticle === "function")
                viewerApp.showBestParticle()
        }
        onReportVariance: {
            if (viewerApp && typeof viewerApp.reportVariance === "function")
                viewerApp.reportVariance()
        }
        onToggleVerbose: {
            if (viewerApp && typeof viewerApp.toggleVerbose === "function")
                viewerApp.toggleVerbose()
        }
        onToggleRedisplay: {
            if (viewerApp && typeof viewerApp.toggleRedisplay === "function")
                viewerApp.toggleRedisplay()
        }
        onCenterView: resetView()
    }

    Column {
        id: overlay
        anchors.left: parent.left
        anchors.leftMargin: 12
        anchors.top: parent.top
        anchors.topMargin: 12
        spacing: 6

        Rectangle {
            color: "#1b2633"
            radius: 4
            opacity: 0.85
            border.color: "#2e3c4d"
            border.width: 1
            width: summaryLabel.implicitWidth + 16
            height: summaryLabel.implicitHeight + 14

            Text {
                id: summaryLabel
                anchors.centerIn: parent
                text: summaryText()
                font.pixelSize: 13
                color: "#d8e6ff"
                wrapMode: Text.Wrap
            }
        }

        Rectangle {
            color: "#1b2633"
            radius: 4
            opacity: 0.85
            border.color: "#2e3c4d"
            border.width: 1
            width: statsLabel.implicitWidth + 16
            height: statsLabel.implicitHeight + 14

            Text {
                id: statsLabel
                anchors.centerIn: parent
                text: qsTr("Particles: %1\nLandmarks: %2")
                      .arg(root.particleModelRef ? root.particleModelRef.count : 0)
                      .arg(root.landmarkModelRef ? root.landmarkModelRef.count : 0)
                font.pixelSize: 13
                color: "#aac4f6"
            }
        }
    }

    function summaryText() {
        if (!root.particleSummaryRef || !root.particleSummaryRef.isValid)
            return qsTr("Particle filter inactive");
        var thetaDeg = root.particleSummaryRef.poseTheta * 180 / Math.PI;
        var zoomText = root.viewStateRef ? root.viewStateRef.zoom.toFixed(2) : "0.00";
        return qsTr("Pose: (%1, %2) @ %3°\nZoom: %4")
            .arg(root.particleSummaryRef.poseX.toFixed(1))
            .arg(root.particleSummaryRef.poseY.toFixed(1))
            .arg(thetaDeg.toFixed(1))
            .arg(zoomText);
    }

    function adjustCenter(dx, dy) {
        if (!root.viewStateRef || !root.viewStateRef.setCenter)
            return;
        var scale = root.viewStateRef.zoom || 0.4;
        var deltaX = dx / scale;
        var deltaY = dy / scale;
        root.viewStateRef.setCenter(root.viewStateRef.centerX + deltaX, root.viewStateRef.centerY + deltaY);
    }

    function adjustZoom(factor) {
        if (!root.viewStateRef || !root.viewStateRef.setZoom)
            return;
        root.viewStateRef.setZoom(root.viewStateRef.zoom * factor);
    }

    function resetView() {
        if (!root.viewStateRef || !root.viewStateRef.setCenter)
            return;
        root.viewStateRef.setCenter(0, 0);
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: keymap.forceActiveFocus()
    }

    Component.onCompleted: keymap.forceActiveFocus()
}
