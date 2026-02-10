import QtQuick 6.5
import QtQuick.Controls 6.5
import QtQuick.Layouts 1.15
import "ui" as UI

FocusScope {
    id: root
    objectName: "depthViewRoot"
    focus: true

    property bool showHelp: false

    implicitWidth: 960
    implicitHeight: 540

    function toggleHelp() {
        showHelp = !showHelp
    }

    Rectangle {
        anchors.fill: parent
        color: "#0b1119"
    }

    UI.DepthKeymap {
        id: keymap
        anchors.fill: parent
        focus: true

        onHelpRequested: root.toggleHelp()
        onToggleLiveRefresh: {
            if (viewerApp && typeof viewerApp.toggleLiveRefresh === "function")
                viewerApp.toggleLiveRefresh()
        }
        onToggleGradientSourceRequested: {
            if (viewerApp && typeof viewerApp.toggleGradientSource === "function")
                viewerApp.toggleGradientSource()
        }
        onRequestSingleRefresh: {
            if (viewerApp && typeof viewerApp.requestSingleRefresh === "function")
                viewerApp.requestSingleRefresh()
        }
        onExitRequested: {
            if (viewerApp && typeof viewerApp.requestQuit === "function")
                viewerApp.requestQuit()
            else
                Qt.quit()
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 12

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#101a27"
            border.color: "#2e3c4d"
            border.width: 1
            radius: 6

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 6

                Text {
                    text: "Depth"
                    color: "#d8e6ff"
                    font.pixelSize: 14
                    font.bold: true
                }

                Image {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    cache: false
                    fillMode: Image.PreserveAspectFit
                    source: "image://depthviz/depth?v=" + depthFrameId
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#101a27"
            border.color: "#2e3c4d"
            border.width: 1
            radius: 6

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 6

                Text {
                    text: {
                        const source = viewerApp && viewerApp.gradientSource ? viewerApp.gradientSource : "camera";
                        return source === "depth" ? "Gradient (Depth)" : "Gradient (Camera)";
                    }
                    color: "#d8e6ff"
                    font.pixelSize: 14
                    font.bold: true
                }

                Image {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    cache: false
                    fillMode: Image.PreserveAspectFit
                    source: "image://depthviz/gradient?v=" + depthFrameId
                }
            }
        }
    }

    Rectangle {
        id: statusBadge
        anchors.left: parent.left
        anchors.leftMargin: 16
        anchors.top: parent.top
        anchors.topMargin: 16
        color: "#1b2633"
        radius: 4
        opacity: 0.9
        border.color: "#2e3c4d"
        border.width: 1
        width: statusLabel.implicitWidth + 18
        height: statusLabel.implicitHeight + 14

        Text {
            id: statusLabel
            anchors.centerIn: parent
            color: "#f3f8ff"
            font.pixelSize: 12
            wrapMode: Text.Wrap
            text: {
                const modeText = (viewerApp && viewerApp.liveEnabled) ? "live@3Hz" : "paused";
                const gradSource = (viewerApp && viewerApp.gradientSource) ? viewerApp.gradientSource : "camera";
                const status = (viewerApp && viewerApp.statusText) ? viewerApp.statusText : "Waiting...";
                return "Depth Viewer (" + modeText + ", grad=" + gradSource + ")\n" + status;
            }
        }
    }

    Rectangle {
        id: helpOverlay
        anchors.fill: parent
        visible: root.showHelp
        color: Qt.rgba(0, 0, 0, 0.74)
        z: 50

        Text {
            anchors.centerIn: parent
            width: parent.width * 0.58
            color: "#f5f5f5"
            text: depthHelpText || ""
            wrapMode: Text.WordWrap
            horizontalAlignment: Text.AlignLeft
        }

        MouseArea {
            anchors.fill: parent
            onClicked: root.showHelp = false
        }
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton
        onPressed: keymap.forceActiveFocus()
    }

    Component.onCompleted: keymap.forceActiveFocus()
}
