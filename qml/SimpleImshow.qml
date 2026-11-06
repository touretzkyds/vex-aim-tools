import QtQuick 6.5

Item {
    id: root
    objectName: "simpleImshow"
    width: 640
    height: 480

    Image {
        id: imageDisplay
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
        cache: false  // CRITICAL: prevent QML from caching the image
        source: "image://imshow/frame?v=" + frameId
    }
}
