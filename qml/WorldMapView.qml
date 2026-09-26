import QtQuick 6.5
import QtQuick3D 6.5
import QtQml.Models


View3D {
    id: worldView
    objectName: "worldView"

    property bool showAxes: true  // Legacy: coordinate axes at origin

    property real moveStepMm: 60
    property real yawStepDeg: 2.5
    property real pitchStepDeg: 2.5
    property real zoomStepGl: 2      // Finer zoom control (was 5)
    property real elevateStepMm: 20
    property real minZoomGl: 2
    property real maxZoomGl: 100
    property real minPitchDeg: -60
    property real maxPitchDeg: 60

    property real sceneYawDeg: 90  // Initial view from right side: camera at -Y, robot faces right (+X)
    property vector3d scenePosition: Qt.vector3d(0, 0, 0)
    property real cameraPitchDeg: -30
    property real cameraDistanceGl: 10
    property vector3d cameraPosition: Qt.vector3d(0, 0, 0)

    onScenePositionChanged: updateCamera()
    onSceneYawDegChanged: updateCamera()
    onCameraPitchDegChanged: updateCamera()
    onCameraDistanceGlChanged: updateCamera()

    readonly property real worldScale: Number(WSCALE) || 0.02

    function worldToSceneVector(xMm, yMm, zMm) {
        const s = worldScale
        return Qt.vector3d(xMm * s, zMm * s, -yMm * s)
    }

    function radiansToDegrees(angle) {
        return angle * 180 / Math.PI
    }

    function degreesToRadians(angle) {
        return angle * Math.PI / 180
    }

    function clamp(value, minValue, maxValue) {
        return Math.max(minValue, Math.min(maxValue, value))
    }

    function wrapDegrees(value) {
        let wrapped = value % 360
        if (wrapped < -180)
            wrapped += 360
        if (wrapped > 180)
            wrapped -= 360
        return wrapped
    }

    function moveScene(forwardMm, strafeMm) {
        const headingRad = degreesToRadians(sceneYawDeg)
        const forwardX = Math.cos(headingRad)
        const forwardY = Math.sin(headingRad)
        const strafeX = -Math.sin(headingRad)
        const strafeY = Math.cos(headingRad)
        const offsetX = forwardMm * forwardX + strafeMm * strafeX
        const offsetY = forwardMm * forwardY + strafeMm * strafeY
        scenePosition = Qt.vector3d(scenePosition.x + offsetX,
                                     scenePosition.y + offsetY,
                                     scenePosition.z)
        updateCamera()
    }

    function yaw(deltaDeg) {
        sceneYawDeg = wrapDegrees(sceneYawDeg + deltaDeg)
        updateCamera()
    }

    function pitch(deltaDeg) {
        cameraPitchDeg = clamp(cameraPitchDeg + deltaDeg, minPitchDeg, maxPitchDeg)
        updateCamera()
    }

    function zoom(deltaGl) {
        cameraDistanceGl = clamp(cameraDistanceGl + deltaGl, minZoomGl, maxZoomGl)
        updateCamera()
    }

    function resetCamera() {
        scenePosition = Qt.vector3d(0, 0, 0)
        sceneYawDeg = 90  // Match initial view: right side view
        cameraPitchDeg = -30
        cameraDistanceGl = 10
        updateCamera()
    }

    function elevate(deltaMm) {
        scenePosition = Qt.vector3d(scenePosition.x,
                                     scenePosition.y,
                                     scenePosition.z + deltaMm)
        updateCamera()
    }

    function zoomIn() { zoom(-zoomStepGl) }
    function zoomOut() { zoom(zoomStepGl) }
    function pitchUp() { pitch(-pitchStepDeg) }
    function pitchDown() { pitch(pitchStepDeg) }
    function toggleAxes() { showAxes = !showAxes }

    function debugSnapshot() {
        const wscale = Number(WSCALE) || 0.02
        const snapshot = {
            ball: { zFinalGL: [] },
            barrel: { items: [] },
            wall: { lengthsGL: [], thicknessGL: 0, heightGL: 0 },
            marker: { thetaDeg: [], sizeGL: [] },
            robot: {}
        }
        const count = worldModel.count || 0
        for (let row = 0; row < count; ++row) {
            const entry = worldModel.get(row)
            if (!entry || entry.missing)
                continue
            const type = entry.type
            if (type === "sports_ball") {
                const zFinal = Number(entry.z) || 0
                snapshot.ball.zFinalGL.push(zFinal * wscale)
            } else if (type === "barrel") {
                const diameter = Number(entry.diameter_mm) || 0
                const height = Number(entry.height_mm) || 0
                snapshot.barrel.items.push({
                    radiusGL: (diameter / 2) * wscale,
                    heightGL: height * wscale
                })
            } else if (type === "apriltag" || type === "aruco") {
                const thetaDeg = radiansToDegrees(Number(entry.theta) || 0)
                const sizeGl = (Number(entry.size_mm) || 0) * wscale
                snapshot.marker.thetaDeg.push(thetaDeg)
                snapshot.marker.sizeGL.push(sizeGl)
            } else if (type === "wall") {
                const lengthGl = (Number(entry.length_mm) || 0) * wscale
                if (lengthGl > 0)
                    snapshot.wall.lengthsGL.push(lengthGl)
                snapshot.wall.thicknessGL = (Number(entry.thickness_mm) || 0) * wscale
                snapshot.wall.heightGL = (Number(entry.height_mm) || 0) * wscale
            } else if (type === "robot") {
                snapshot.robot.thetaDeg = radiansToDegrees(Number(entry.theta) || 0)
                snapshot.robot.holdingState = Boolean(entry.holding)
            }
        }
        return snapshot
    }

    function updateCamera() {
        if (!worldCamera)
            return

        const clampedDistance = clamp(cameraDistanceGl, minZoomGl, maxZoomGl)
        if (clampedDistance !== cameraDistanceGl)
            cameraDistanceGl = clampedDistance

        const yawRad = degreesToRadians(sceneYawDeg)
        const pitchRad = degreesToRadians(cameraPitchDeg)
        const distanceMm = clampedDistance / worldScale
        const cosPitch = Math.cos(pitchRad)
        const sinPitch = Math.sin(pitchRad)
        const cosYaw = Math.cos(yawRad)
        const sinYaw = Math.sin(yawRad)

        const relX = -cosPitch * cosYaw * distanceMm
        const relY = -cosPitch * sinYaw * distanceMm
        const relZ = -sinPitch * distanceMm

        const focusGl = worldToSceneVector(scenePosition.x, scenePosition.y, scenePosition.z)
        const offsetGl = worldToSceneVector(relX, relY, relZ)

        cameraPosition = Qt.vector3d(
            focusGl.x + offsetGl.x,
            focusGl.y + offsetGl.y,
            focusGl.z + offsetGl.z
        )

        if (worldCamera.lookAt) {
            worldCamera.lookAt(focusGl)
        } else {
            worldCamera.eulerRotation = Qt.vector3d(cameraPitchDeg, sceneYawDeg, 0)
        }
    }

    focus: true
    Component.onCompleted: {
        worldView.forceActiveFocus()
        updateCamera()
    }

    environment: SceneEnvironment {
        clearColor: Qt.rgba(0.1, 0.1, 0.12, 1.0)  // Brighter background
        backgroundMode: SceneEnvironment.Color
        antialiasingMode: SceneEnvironment.MSAA
        antialiasingQuality: SceneEnvironment.High
    }

    camera: PerspectiveCamera {
        id: worldCamera
        objectName: "worldCamera"
        fieldOfView: 50
        clipNear: 0.1   // Adjusted for smaller object scale (was 5)
        clipFar: 200    // Adjusted for closer camera distances (was 600)
        position: cameraPosition
    }

    Node {
        id: sceneRoot
        objectName: "sceneRoot"
        position: Qt.vector3d(0, 0, 0)

        Node {
            id: sceneBasis
            rotation: Qt.quaternion(0.7071, -0.7071, 0, 0)
            scale: Qt.vector3d(worldScale, worldScale, worldScale)

            Node {
                id: sceneFrame
                objectName: "sceneFrame"

                // Ambient light to reduce harsh shading
                DirectionalLight {
                    brightness: 0.3
                    ambientColor: Qt.rgba(0.4, 0.4, 0.4, 1)
                    eulerRotation: Qt.vector3d(0, 0, 0)
                    castsShadow: false
                }

                DirectionalLight {
                    brightness: 0.6  // Increased brightness for better visibility
                    eulerRotation: Qt.vector3d(-50, 35, 0)
                    castsShadow: false
                }

                DirectionalLight {
                    brightness: 0.4  // Increased brightness for better visibility
                    eulerRotation: Qt.vector3d(20, -145, 0)
                    castsShadow: false
                }

                Model {
                    id: ground
                    source: "#Cube"
                    position: Qt.vector3d(0, 0, -25)
                    // Qt Quick 3D #Cube: edge length=100. Ground plane: 20m x 20m x 50mm in real world.
                    scale: Qt.vector3d(200, 200, 0.5)
                    materials: PrincipledMaterial {
                        baseColor: Qt.rgba(0.25, 0.25, 0.25, 1)  // Legacy: color_gray * 0.5 (highlight=None darkens)
                        roughness: 0.9
                        metalness: 0.0
                    }
                }

                // Floor grid lines - extended range (100mm spacing, ±3000mm range for better visibility)
                Node {
                    id: floorGrid
                    visible: true
                    
                    // X-direction lines (parallel to X-axis, varying in Y)
                    Repeater3D {
                        model: 61  // -3000 to +3000, step 100mm = 61 lines
                        Model {
                            source: "#Cube"
                            property real yPosMm: (index - 30) * 100  // -3000 to +3000, origin at grid intersection
                            position: Qt.vector3d(0, yPosMm, 0)
                            // Line: 6000mm long (X), 2mm wide (Y), 1mm high (Z), slightly above ground
                            scale: Qt.vector3d(60, 0.02, 0.01)
                            materials: PrincipledMaterial {
                                baseColor: Qt.rgba(0.65, 0.65, 0.65, 1)  // Legacy color_light_gray
                                roughness: 0.5
                                metalness: 0.0
                                cullMode: Material.NoCulling
                            }
                        }
                    }
                    
                    // Y-direction lines (parallel to Y-axis, varying in X)
                    Repeater3D {
                        model: 61  // -3000 to +3000, step 100mm = 61 lines
                        Model {
                            source: "#Cube"
                            property real xPosMm: (index - 30) * 100   // -3000 to +3000, origin at grid intersection
                            position: Qt.vector3d(xPosMm, 0, 0)
                            // Line: 2mm wide (X), 6000mm long (Y), 1mm high (Z), slightly above ground
                            scale: Qt.vector3d(0.02, 60, 0.01)
                            materials: PrincipledMaterial {
                                baseColor: Qt.rgba(0.65, 0.65, 0.65, 1)  // Legacy color_light_gray
                                roughness: 0.5
                                metalness: 0.0
                                cullMode: Material.NoCulling
                            }
                        }
                    }
                }

                Node {
                    id: axes
                    visible: showAxes
                    readonly property real axisLengthMm: 100  // 100mm (10cm) matching legacy
                    readonly property real axisThicknessMm: 2   // 1mm thin lines matching legacy

                    // X-axis (red) - extends from origin to +X direction
                    Model {
                        source: "#Cube"
                        scale: Qt.vector3d(axes.axisLengthMm / 100, axes.axisThicknessMm / 100, axes.axisThicknessMm / 100)
                        position: Qt.vector3d(50, 0, 0)  // Translate by len/2 to extend from origin
                        materials: PrincipledMaterial {
                            baseColor: "#ff0000"  // Legacy color_red (1, 0, 0)
                            cullMode: Material.NoCulling
                        }
                    }

                    // Y-axis (green) - extends from origin to +Y direction
                    Model {
                        source: "#Cube"
                        scale: Qt.vector3d(axes.axisThicknessMm / 100, axes.axisLengthMm / 100, axes.axisThicknessMm / 100)
                        position: Qt.vector3d(0, 50, 0)  // Translate by len/2 to extend from origin
                        materials: PrincipledMaterial {
                            baseColor: "#00ff00"  // Legacy color_green (0, 1, 0)
                            cullMode: Material.NoCulling
                        }
                    }

                    // Z-axis (blue) - extends from origin to +Z direction
                    Model {
                        source: "#Cube"
                        scale: Qt.vector3d(axes.axisThicknessMm / 100, axes.axisThicknessMm / 100, axes.axisLengthMm / 100)
                        position: Qt.vector3d(0, 0, 50)  // Translate by len/2 to extend from origin
                        materials: PrincipledMaterial {
                            baseColor: "#4770f2"  // Legacy color_blue (0.28, 0.44, 0.95)
                            cullMode: Material.NoCulling
                        }
                    }
                }
            }
        }
    }

    Component {
        id: robotDelegate
        Node {
            property var model
            parent: sceneFrame
            visible: !model.missing
            // VEX robot: radius=32mm, height=72mm (legacy make_vex_robot)
            readonly property real radiusMm: 32
            readonly property real heightMm: 72
            // Robot cylinder is centered, so lift it by half height to sit on ground
            position: Qt.vector3d(model.x, model.y, model.z + heightMm / 2)
            eulerRotation.z: radiansToDegrees(model.theta)

            // Main body: light gray cylinder (legacy color_light_gray = 0.65, 0.65, 0.65)
            Model {
                source: "#Cylinder"
                eulerRotation.x: 90
                // Qt Quick 3D #Cylinder: diameter=100, height=100. Scale to mm, then worldScale handles GL conversion.
                scale: Qt.vector3d(radiusMm / 50, heightMm / 100, radiusMm / 50)
                materials: PrincipledMaterial {
                    baseColor: "#a6a6a6"  // light_gray (0.65, 0.65, 0.65)
                    roughness: 0.4
                    metalness: 0.1
                    cullMode: Material.NoCulling
                }
            }

            // Direction indicator: small black sphere (legacy: radius=12mm at position 30,0,42)
            Model {
                source: "#Sphere"
                // Position: forward 30mm (X-axis in local coords after sceneBasis rotation), height 42mm absolute
                // Note: sceneBasis rotation transforms world coords; in local robot frame, +X is forward
                position: Qt.vector3d(30, 0, 42 - heightMm / 2)
                // Qt Quick 3D #Sphere: diameter=100. Scale to 24mm diameter (12mm radius).
                scale: Qt.vector3d(0.24, 0.24, 0.24)
                materials: PrincipledMaterial {
                    baseColor: "#000000"  // color_black
                    roughness: 0.3
                    metalness: 0.0
                    cullMode: Material.NoCulling
                }
            }
        }
    }

    Component {
        id: ballDelegate
        Node {
            id: ballRoot
            property var model
            parent: sceneFrame
            readonly property real radiusMm: (model.diameter_mm || 25) / 2
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                visible: !model.missing
                source: "#Sphere"
                scale: Qt.vector3d(parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100)
                materials: PrincipledMaterial {
                    baseColor: model.visible ? "#e6b319" : "#8a6b0f"
                    roughness: 0.3
                    cullMode: Material.NoCulling
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.08, 0.01) : Qt.vector3d(0, 0, 0)
                }
            }

            Node {
                id: ballWireframe
                visible: ballRoot.model.missing
                readonly property int circleSegments: 20
                readonly property real lineThick: 0.8

                PrincipledMaterial {
                    id: ballWireMat
                    baseColor: "#8a6b0f"
                    roughness: 0.3
                    cullMode: Material.NoCulling
                }

                Repeater3D {
                    model: [
                        Qt.vector3d(0, 0, 0),
                        Qt.vector3d(90, 0, 0),
                        Qt.vector3d(90, 0, 45),
                        Qt.vector3d(90, 0, 90),
                        Qt.vector3d(90, 0, 135),
                        Qt.vector3d(45, 0, 0),
                        Qt.vector3d(45, 0, 60),
                        Qt.vector3d(45, 0, 120)
                    ]
                    Node {
                        eulerRotation: modelData
                        Repeater3D {
                            model: ballWireframe.circleSegments
                            Model {
                                source: "#Cube"
                                readonly property real midAngle: (index + 0.5) * 2 * Math.PI / ballWireframe.circleSegments
                                readonly property real chord: 2 * ballRoot.radiusMm * Math.sin(Math.PI / ballWireframe.circleSegments)
                                position: Qt.vector3d(
                                    ballRoot.radiusMm * Math.cos(midAngle),
                                    ballRoot.radiusMm * Math.sin(midAngle),
                                    0
                                )
                                eulerRotation.z: midAngle * 180 / Math.PI
                                scale: Qt.vector3d(ballWireframe.lineThick / 100, chord / 100, ballWireframe.lineThick / 100)
                                materials: ballWireMat
                            }
                        }
                    }
                }
            }
        }
    }

    Component {
        id: barrelDelegate
        Node {
            id: barrelRoot
            property var model
            parent: sceneFrame
            readonly property real radiusMm: (model.diameter_mm || 22) / 2
            readonly property real heightMm: model.height_mm || 25
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                visible: !model.missing
                source: "#Cylinder"
                eulerRotation.x: 90
                scale: Qt.vector3d(parent.radiusMm / 50, parent.heightMm / 100, parent.radiusMm / 50)
                materials: PrincipledMaterial {
                    baseColor: {
                        if (model.type === "barrel_orange")
                            return model.visible ? "#ff8010" : "#994d0a"
                        if (model.type === "barrel_blue")
                            return model.visible ? "#4770f2" : "#2a4391"
                        return model.visible ? "#18d0ff" : "#0e7e97"
                    }
                    roughness: 0.45
                    cullMode: Material.NoCulling
                    emissiveFactor: {
                        if (!model.visible)
                            return Qt.vector3d(0, 0, 0)
                        if (model.type === "barrel_orange")
                            return Qt.vector3d(0.15, 0.08, 0.01)
                        if (model.type === "barrel_blue")
                            return Qt.vector3d(0.05, 0.08, 0.18)
                        return Qt.vector3d(0.06, 0.2, 0.25)
                    }
                }
            }

            Node {
                id: barrelWireframe
                visible: barrelRoot.model.missing
                readonly property int rimSegments: 8
                readonly property int strutCount: 8
                readonly property real lineThick: 2.5 // 0.8

                PrincipledMaterial {
                    id: barrelWireMat
                    baseColor: {
                        if (barrelRoot.model.type === "barrel_orange")
                            return "#ed8528" // "#994d0a"
                        if (barrelRoot.model.type === "barrel_blue")
                            return "#5879e0" // "#2a4391"
                        return "#0e7e97"
                    }
                    roughness: 0.3
                    cullMode: Material.NoCulling
                }

                Repeater3D {
                    model: barrelWireframe.rimSegments * 2
                    Model {
                        source: "#Cube"
                        readonly property int ringIdx: index % barrelWireframe.rimSegments
                        readonly property real zSign: index < barrelWireframe.rimSegments ? 1 : -1
                        readonly property real midAngle: (ringIdx + 0.5) * 2 * Math.PI / barrelWireframe.rimSegments
                        readonly property real chord: 2 * barrelRoot.radiusMm * Math.sin(Math.PI / barrelWireframe.rimSegments)
                        position: Qt.vector3d(
                            barrelRoot.radiusMm * Math.cos(midAngle),
                            barrelRoot.radiusMm * Math.sin(midAngle),
                            zSign * barrelRoot.heightMm / 2
                        )
                        eulerRotation.z: midAngle * 180 / Math.PI
                        scale: Qt.vector3d(barrelWireframe.lineThick / 100, chord / 100, barrelWireframe.lineThick / 100)
                        materials: barrelWireMat
                    }
                }

                Repeater3D {
                    model: barrelWireframe.strutCount
                    Model {
                        source: "#Cube"
                        readonly property real strutAngle: index * 2 * Math.PI / barrelWireframe.strutCount
                        position: Qt.vector3d(
                            barrelRoot.radiusMm * Math.cos(strutAngle),
                            barrelRoot.radiusMm * Math.sin(strutAngle),
                            0
                        )
                        scale: Qt.vector3d(barrelWireframe.lineThick / 100, barrelWireframe.lineThick / 100, barrelRoot.heightMm / 100)
                        materials: barrelWireMat
                    }
                }

                Repeater3D {
                    model: barrelWireframe.rimSegments * 2
                    Model {
                        source: "#Cube"
                        readonly property int spokeIdx: index % barrelWireframe.rimSegments
                        readonly property real zSign: index < barrelWireframe.rimSegments ? 1 : -1
                        readonly property real spokeAngle: spokeIdx * 2 * Math.PI / barrelWireframe.rimSegments
                        position: Qt.vector3d(
                            barrelRoot.radiusMm / 2 * Math.cos(spokeAngle),
                            barrelRoot.radiusMm / 2 * Math.sin(spokeAngle),
                            zSign * barrelRoot.heightMm / 2
                        )
                        eulerRotation.z: spokeAngle * 180 / Math.PI
                        scale: Qt.vector3d(barrelRoot.radiusMm / 100, barrelWireframe.lineThick / 100, barrelWireframe.lineThick / 100)
                        materials: barrelWireMat
                    }
                }
            }
        }
    }

    Component {
        id: openvocabDelegate
        Node {
            id: openvocabRoot
            property var model
            parent: sceneFrame
            // Approximate dimensions estimated from the camera bounding box.
            readonly property real radiusMm: (model.diameter_mm || 40) / 2
            readonly property real heightMm: model.height_mm || 40
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            readonly property color sampledColor: model.color || "#35d07f"

            Model {
                source: "#Cylinder"
                eulerRotation.x: 90
                scale: Qt.vector3d(parent.radiusMm / 50, parent.heightMm / 100, parent.radiusMm / 50)
                opacity: model.missing ? 0.35 : 1.0
                materials: PrincipledMaterial {
                    baseColor: model.visible ? openvocabRoot.sampledColor
                                             : Qt.darker(openvocabRoot.sampledColor, 1.7)
                    roughness: 0.45
                    cullMode: Material.NoCulling
                    emissiveFactor: model.visible ? Qt.vector3d(0.05, 0.05, 0.05)
                                                  : Qt.vector3d(0, 0, 0)
                }
            }
        }
    }

    Component {
        id: markerDelegate
        Node {
            id: markerRoot
            property var model
            parent: sceneFrame
            readonly property real widthMm: model.width_mm || 38
            readonly property real heightMm: model.height_mm || 48
            readonly property real thicknessMm: model.thickness_mm || 2
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                visible: !model.missing
                source: "#Cube"
                scale: Qt.vector3d(thicknessMm / 100, widthMm / 100, heightMm / 100)
                materials: PrincipledMaterial {
                    baseColor: {
                        if (model.type === "apriltag")
                            return "#804ce6";
                        if (model.type === "aruco")
                            return model.visible ? "#00ff00" : "#1f431f";
                        return "#202020";
                    }
                    emissiveFactor: {
                        if (!model.visible)
                            return Qt.vector3d(0.02, 0.02, 0.02);
                        if (model.type === "apriltag")
                            return Qt.vector3d(0.08, 0.05, 0.15);
                        if (model.type === "aruco")
                            return Qt.vector3d(0.08, 0.16, 0.08);
                        return Qt.vector3d(0.1, 0.1, 0.1);
                    }
                    roughness: 0.2
                    cullMode: Material.NoCulling
                }
            }

            Image {
                id: tagImageFront
                source: (model.type === "apriltag" || model.type === "aruco")
                        ? "image://tagtexture/" + (model.type === "aruco" ? "aruco-" : "") + String(model.marker_id)
                        : ""
                visible: false
                cache: true
            }

            Image {
                id: tagImageBack
                source: (model.type === "apriltag" || model.type === "aruco")
                        ? "image://tagtexture/back-" + (model.type === "aruco" ? "aruco-" : "") + String(model.marker_id)
                        : ""
                visible: false
                cache: true
            }
            
            Model {
                id: textPanelFront
                visible: !model.missing
                         && (model.type === "apriltag" || model.type === "aruco")
                         && model.marker_id !== null && model.marker_id !== undefined && model.marker_id !== ""
                source: "#Cube"
                scale: Qt.vector3d(0.005, heightMm / 100, widthMm / 100)
                position: Qt.vector3d(thicknessMm / 2 + 0.5, 0, 0)
                eulerRotation: Qt.vector3d(90, 0, 0)
                
                materials: PrincipledMaterial {
                    baseColor: "#ffffff"
                    baseColorMap: Texture {
                        sourceItem: tagImageFront
                    }
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.06, 0.2) : Qt.vector3d(0, 0, 0)
                    roughness: 0.1
                    cullMode: Material.NoCulling
                }
                opacity: model.visible ? 1.0 : 0.8
            }
            
            Model {
                id: textPanelBack
                visible: !model.missing
                         && (model.type === "apriltag" || model.type === "aruco")
                         && model.marker_id !== null && model.marker_id !== undefined && model.marker_id !== ""
                source: "#Cube"
                scale: Qt.vector3d(0.005, heightMm / 100, widthMm / 100)
                position: Qt.vector3d(-thicknessMm / 2 - 0.5, 0, 0)
                eulerRotation: Qt.vector3d(90, 0, 0)
                
                materials: PrincipledMaterial {
                    baseColor: "#ffffff"
                    baseColorMap: Texture {
                        sourceItem: tagImageBack
                    }
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.06, 0.2) : Qt.vector3d(0, 0, 0)
                    roughness: 0.1
                    cullMode: Material.NoCulling
                }
                opacity: model.visible ? 1.0 : 0.8
            }

            Node {
                id: markerWireframe
                visible: markerRoot.model.missing
                readonly property real lineThick: 0.8

                PrincipledMaterial {
                    id: markerWireMat
                    baseColor: {
                        if (markerRoot.model.type === "apriltag")
                            return "#804ce6"
                        if (markerRoot.model.type === "aruco")
                            return "#1f431f"
                        return "#202020"
                    }
                    roughness: 0.3
                    cullMode: Material.NoCulling
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(0, 0, markerRoot.heightMm / 2)
                    scale: Qt.vector3d(markerWireframe.lineThick / 100, markerRoot.widthMm / 100, markerWireframe.lineThick / 100)
                    materials: markerWireMat
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(0, 0, -markerRoot.heightMm / 2)
                    scale: Qt.vector3d(markerWireframe.lineThick / 100, markerRoot.widthMm / 100, markerWireframe.lineThick / 100)
                    materials: markerWireMat
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(0, -markerRoot.widthMm / 2, 0)
                    scale: Qt.vector3d(markerWireframe.lineThick / 100, markerWireframe.lineThick / 100, markerRoot.heightMm / 100)
                    materials: markerWireMat
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(0, markerRoot.widthMm / 2, 0)
                    scale: Qt.vector3d(markerWireframe.lineThick / 100, markerWireframe.lineThick / 100, markerRoot.heightMm / 100)
                    materials: markerWireMat
                }
            }
        }
    }

    Component {
        id: wallDelegate
        Node {
            property var model
            parent: sceneFrame
            visible: !model.missing
            readonly property real lengthMm: model.length_mm || 300
            readonly property real heightMm: model.height_mm || 210
            readonly property real thicknessMm: model.thickness_mm || 4
            readonly property var doorways: (model.doorways && model.doorways.length) ? model.doorways : []
            readonly property real wallBaseLocalZ: -heightMm / 2
            readonly property real doorHeightMm: {
                if (!doorways.length)
                    return 0
                var maxHeight = 0
                for (var i = 0; i < doorways.length; ++i) {
                    var h = Number(doorways[i].height) || 0
                    if (h > maxHeight)
                        maxHeight = h
                }
                return Math.max(0, Math.min(heightMm, maxHeight))
            }
            readonly property real lowerHeightMm: doorHeightMm > 0 ? doorHeightMm : heightMm
            readonly property var lowerSegments: computeLowerSegments()
            // Use model.z which is already computed in Python (height/2)
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            function computeLowerSegments() {
                if (!doorways.length) {
                    return [{ "center": 0, "width": lengthMm }]
                }

                var cursor = -lengthMm / 2
                var end = lengthMm / 2
                var segments = []
                var sorted = []
                for (var i = 0; i < doorways.length; ++i)
                    sorted.push(doorways[i])
                sorted.sort(function(a, b) { return Number(a.x) - Number(b.x) })

                for (var j = 0; j < sorted.length; ++j) {
                    var spec = sorted[j]
                    var centerY = (Number(spec.x) || 0) - lengthMm / 2
                    var width = Math.max(0, Number(spec.width) || 0)
                    var left = centerY - width / 2
                    var right = centerY + width / 2
                    if (left > cursor) {
                        var segWidth = left - cursor
                        segments.push({
                            "center": cursor + segWidth / 2,
                            "width": segWidth
                        })
                    }
                    cursor = Math.max(cursor, right)
                }

                if (cursor < end) {
                    var tailWidth = end - cursor
                    segments.push({
                        "center": cursor + tailWidth / 2,
                        "width": tailWidth
                    })
                }
                return segments
            }

            PrincipledMaterial {
                id: wallMaterial
                baseColor: model.visible ? Qt.rgba(0.88, 0.78, 0.22, 0.74) : Qt.rgba(0.55, 0.48, 0.14, 0.60)
                alphaMode: PrincipledMaterial.Blend
                roughness: 0.65
                cullMode: Material.NoCulling
                emissiveFactor: model.visible ? Qt.vector3d(0.06, 0.06, 0.02) : Qt.vector3d(0.02, 0.02, 0.01)
            }

            Repeater3D {
                model: lowerSegments.length
                Model {
                    property int idx: index
                    source: "#Cube"
                    property var seg: lowerSegments[idx]
                    position: Qt.vector3d(0, Number(seg.center) || 0, wallBaseLocalZ + lowerHeightMm / 2)
                    scale: Qt.vector3d(thicknessMm / 100, (Number(seg.width) || 0) / 100, lowerHeightMm / 100)
                    materials: wallMaterial
                }
            }

            Model {
                visible: doorHeightMm > 0 && doorHeightMm < heightMm
                source: "#Cube"
                property real transomHeightMm: Math.max(0, heightMm - doorHeightMm)
                position: Qt.vector3d(0, 0, wallBaseLocalZ + doorHeightMm + transomHeightMm / 2)
                scale: Qt.vector3d(thicknessMm / 100, lengthMm / 100, transomHeightMm / 100)
                materials: wallMaterial
            }
        }
    }

    Component {
        id: worldDelegate
        Node {
            id: worldDelegateRoot
            property var modelSnapshot: model
            property Node delegateItem
            property Component delegateComponent: null

            function componentForType(typeName) {
                switch (typeName) {
                case "robot":
                    return robotDelegate
                case "sports_ball":
                    return ballDelegate
                case "barrel":
                case "barrel_orange":
                case "barrel_blue":
                    return barrelDelegate
                case "apriltag":
                case "aruco":
                    return markerDelegate
                case "openvocab":
                    return openvocabDelegate
                case "wall":
                    return wallDelegate
                default:
                    return null
                }
            }

            function rebuild() {
                const data = modelSnapshot
                if (!data || !data.type) {
                    if (delegateItem) {
                        delegateItem.destroy()
                        delegateItem = null
                    }
                    delegateComponent = null
                    return
                }

                const component = componentForType(data.type)
                if (delegateComponent === component && delegateItem) {
                    if (delegateItem.model !== data)
                        delegateItem.model = data
                    return
                }

                if (delegateItem) {
                    delegateItem.destroy()
                    delegateItem = null
                }
                delegateComponent = component
                if (!component)
                    return

                delegateItem = component.createObject(sceneFrame, {
                    "model": data
                })
                if (!delegateItem)
                    console.warn("Failed to create delegate for type", data.type)
            }

            onModelSnapshotChanged: rebuild()
            Component.onDestruction: {
                if (delegateItem) {
                    delegateItem.destroy()
                    delegateItem = null
                }
                delegateComponent = null
            }
        }
    }

    Repeater3D {
        id: worldRepeater
        parent: sceneFrame
        model: worldModel
        delegate: worldDelegate
    }

    // Screen-space labels remain legible as the camera moves or zooms.
    Repeater {
        model: worldModel
        delegate: Text {
            readonly property vector3d scenePos:
                sceneFrame.mapPositionToScene(Qt.vector3d(model.x, model.y,
                                                          model.z + (model.height_mm || 40) / 2 + 20))
            readonly property vector3d screenPos: worldView.mapFrom3DScene(scenePos)

            visible: model.type === "openvocab" && screenPos.z > 0
            text: model.label || ""
            x: screenPos.x - width / 2
            y: screenPos.y - height
            color: model.missing ? "#c08a6a" : "#ffffff"
            font.pixelSize: 13
            font.bold: true
            style: Text.Outline
            styleColor: "#101010"
        }
    }

    Keys.onPressed: function(event) {
        switch (event.key) {
        case Qt.Key_W:
            moveScene(moveStepMm, 0)
            event.accepted = true
            break
        case Qt.Key_S:
            moveScene(-moveStepMm, 0)
            event.accepted = true
            break
        case Qt.Key_A:
            moveScene(0, moveStepMm)
            event.accepted = true
            break
        case Qt.Key_D:
            moveScene(0, -moveStepMm)
            event.accepted = true
            break
        case Qt.Key_J:
            yaw(-yawStepDeg)
            event.accepted = true
            break
        case Qt.Key_L:
            yaw(yawStepDeg)
            event.accepted = true
            break
        case Qt.Key_Left:
            yaw(-yawStepDeg)
            event.accepted = true
            break
        case Qt.Key_Right:
            yaw(yawStepDeg)
            event.accepted = true
            break
        case Qt.Key_I:
            pitchUp()
            event.accepted = true
            break
        case Qt.Key_K:
            pitchDown()
            event.accepted = true
            break
        case Qt.Key_Up:
            pitchUp()
            event.accepted = true
            break
        case Qt.Key_Down:
            pitchDown()
            event.accepted = true
            break
        case Qt.Key_Comma:
        case Qt.Key_Less:
            zoomIn()
            event.accepted = true
            break
        case Qt.Key_Period:
        case Qt.Key_Greater:
            zoomOut()
            event.accepted = true
            break
        case Qt.Key_Minus:
        case Qt.Key_Underscore:
            zoomOut()
            event.accepted = true
            break
        case Qt.Key_Equal:
        case Qt.Key_Plus:
            zoomIn()
            event.accepted = true
            break
        case Qt.Key_PageUp:
            elevate(elevateStepMm)
            event.accepted = true
            break
        case Qt.Key_PageDown:
            elevate(-elevateStepMm)
            event.accepted = true
            break
        case Qt.Key_Q:
            elevate(elevateStepMm)
            event.accepted = true
            break
        case Qt.Key_E:
            elevate(-elevateStepMm)
            event.accepted = true
            break
        case Qt.Key_Z:
            resetCamera()
            event.accepted = true
            break
        case Qt.Key_X:
            toggleAxes()
            event.accepted = true
            break
        case Qt.Key_H:
            if (typeof viewerApp !== "undefined" && viewerApp && viewerApp.printHelp) {
                viewerApp.printHelp()
            } else {
                console.log("World viewer keyboard commands: w/a/s/d translate, q/e or PgUp/PgDn raise/lower, </> zoom, i/k or arrows pitch, j/l or arrows yaw, x toggle axes, z reset")
            }
            event.accepted = true
            break
        default:
            break
        }
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton
        onPressed: worldView.forceActiveFocus()
    }
}
