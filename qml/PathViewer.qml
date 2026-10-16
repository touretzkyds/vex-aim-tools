import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "layers"
import "ui" as UI

FocusScope {
    id: root
    objectName: "pathViewRoot"
    focus: true

    property var viewStateRef: viewState
    property var nodeModel: pathNodeModel
    property var edgeModel: pathEdgeModel
    property var overlayModel: pathOverlayModel
    property var obstacleModel: pathObstacleModel
    property var robotModel: pathRobotModel

    Rectangle {
        anchors.fill: parent
        color: "#0b1119"
    }

    SplitView {
        id: splitView
        anchors.fill: parent
        orientation: Qt.Horizontal
        handle: Rectangle {
            implicitWidth: 6
            color: "#15202c"
        }

        PathCanvas {
            id: pathCanvas
            objectName: "pathCanvas"
            implicitWidth: 380
            viewState: root.viewStateRef
            nodeModel: root.nodeModel
            edgeModel: root.edgeModel
            overlayModel: root.overlayModel
            obstacleModel: root.obstacleModel
            robotModel: root.robotModel
        }

        WavefrontView {
            id: wavefrontView
            implicitWidth: 280
            source: viewerApp ? viewerApp.wavefrontSource : ""
            squareSizeMm: viewerApp ? viewerApp.wavefrontSquareSize : 5.0
            originX: viewerApp ? viewerApp.wavefrontOriginX : 0.0
            originY: viewerApp ? viewerApp.wavefrontOriginY : 0.0
            viewState: root.viewStateRef
        }
    }

    UI.PathKeymap {
        id: keymap
        objectName: "pathKeymap"
        anchors.fill: parent
        focus: true

        onViewPan: function(deltaX, deltaY) { adjustCenter(deltaX, deltaY); }
        onViewZoom: function(factor) { adjustZoom(factor); }
        onHelpRequested: {
            if (viewerApp && typeof viewerApp.printHelp === "function")
                viewerApp.printHelp();
        }
        onToggleRedisplay: {
            if (viewerApp && typeof viewerApp.toggleRedisplay === "function")
                viewerApp.toggleRedisplay();
        }
        onRequestRefresh: {
            if (viewerApp && typeof viewerApp.requestRefresh === "function")
                viewerApp.requestRefresh();
        }
        onShowObjects: {
            if (viewerApp && typeof viewerApp.showObjects === "function")
                viewerApp.showObjects();
        }
        onShowObstacles: {
            if (viewerApp && typeof viewerApp.showObstacles === "function")
                viewerApp.showObstacles();
        }
        onShowPose: {
            if (viewerApp && typeof viewerApp.showPose === "function")
                viewerApp.showPose();
        }
        onCenterView: resetView();
        onExitRequested: {
            if (viewerApp && typeof viewerApp.requestQuit === "function")
                viewerApp.requestQuit();
            else
                Qt.quit();
        }
    }

    Column {
        id: overlays
        anchors.left: parent.left
        anchors.leftMargin: 12
        anchors.top: parent.top
        anchors.topMargin: 12
        spacing: 6

        Rectangle {
            visible: viewerApp && viewerApp.statusText.length > 0
            color: "#1b2633"
            radius: 4
            opacity: 0.85
            border.color: "#2e3c4d"
            border.width: 1
            width: statusLabel.implicitWidth + 16
            height: statusLabel.implicitHeight + 14

            Text {
                id: statusLabel
                anchors.centerIn: parent
                text: viewerApp ? viewerApp.statusText : ""
                font.pixelSize: 13
                color: "#f6d47d"
                wrapMode: Text.Wrap
            }
        }
    }

    Component.onCompleted: keymap.forceActiveFocus()

    function adjustCenter(dx, dy) {
        if (!root.viewStateRef || !root.viewStateRef.setCenter)
            return;
        var scale = root.viewStateRef.zoom || 0.64;
        var deltaX = dx / scale;
        var deltaY = dy / scale;
        root.viewStateRef.setCenter(root.viewStateRef.centerX + deltaY, root.viewStateRef.centerY - deltaX);
    }

    function adjustZoom(factor) {
        if (!root.viewStateRef || !root.viewStateRef.setZoom)
            return;
        root.viewStateRef.setZoom(root.viewStateRef.zoom * factor);
    }

    function resetView() {
        if (viewerApp && typeof viewerApp.centerView === "function")
            viewerApp.centerView();
        if (root.viewStateRef && root.viewStateRef.setCenter)
            root.viewStateRef.setCenter(0, 0);
    }
}
