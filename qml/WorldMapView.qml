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

    // Grid frame counter (NO binding - updated via function call from Python)
    property int localGridFrameId: 0

    // Function to update grid frame (called from Python, breaks binding loop)
    function updateGridFrame(newId) {
        if (localGridFrameId !== newId) {
            localGridFrameId = newId
        }
    }

    onScenePositionChanged: updateCamera()
    onSceneYawDegChanged: updateCamera()
    onCameraPitchDegChanged: updateCamera()
    onCameraDistanceGlChanged: updateCamera()

    readonly property real worldScale: Number(WSCALE) || 0.02

    function worldToSceneVector(xMm, yMm, zMm) {
        const s = worldScale
        return Qt.vector3d(-yMm * s, zMm * s, -xMm * s)
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
            rotation: Qt.quaternion(0.5, -0.5, 0.5, 0.5)
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

                // Occupancy Grid Visualization
                // Hidden 2D image pulls from the image provider; Quick3D then uses it as a texture.
                Image {
                    id: gridImage
                    source: "image://grid/map?v=" + worldView.localGridFrameId
                    visible: false
                    // Use default caching; versioned source busts cache when gridFrameId changes
                    smooth: false
                    asynchronous: true  // Prevent UI blocking during texture load
                }

                Model {
                    id: occupancyGrid
                    source: "#Rectangle"
                    
                    property real gMinX: typeof GRID_X_MIN !== "undefined" ? GRID_X_MIN : -2500
                    property real gMinY: typeof GRID_Y_MIN !== "undefined" ? GRID_Y_MIN : -2500
                    property real gWidth: typeof GRID_WIDTH_MM !== "undefined" ? GRID_WIDTH_MM : 5000
                    property real gHeight: typeof GRID_HEIGHT_MM !== "undefined" ? GRID_HEIGHT_MM : 5000
                    
                    property real cx: gMinX + gWidth / 2
                    property real cy: gMinY + gHeight / 2
                    
                    // Place just above the ground plane (top at z=0) to keep it visible
                    position: Qt.vector3d(cx, cy, 1)
                    
                    // #Rectangle is 100x100. Scale to grid size.
                    scale: Qt.vector3d(gWidth / 100, gHeight / 100, 1)
                    
                    materials: PrincipledMaterial {
                        baseColor: "#ffffff"
                        baseColorMap: Texture {
                            sourceItem: gridImage
                        }
                        alphaMode: PrincipledMaterial.Blend
                        cullMode: Material.NoCulling
                        lighting: PrincipledMaterial.NoLighting
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
                    readonly property real axisThicknessMm: 2   // Slightly thicker lines for better visibility

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
            property var model
            visible: !model.missing
            readonly property real radiusMm: (model.diameter_mm || 25) / 2
            // Use model.z which is already computed in Python (diameter/2)
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                source: "#Sphere"
                // Qt Quick 3D #Sphere: diameter=100. Scale to mm diameter.
                scale: Qt.vector3d(parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100)
                materials: PrincipledMaterial {
                    // Legacy ball color: (0.9, 0.7, 0.1) = #e6b319 yellow
                    baseColor: model.visible ? "#e6b319" : "#8a6b0f"
                    roughness: 0.3
                    cullMode: Material.NoCulling
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.08, 0.01) : Qt.vector3d(0, 0, 0)
                }
            }
        }
    }

    Component {
        id: barrelDelegate
        Node {
            property var model
            visible: !model.missing
            readonly property real radiusMm: (model.diameter_mm || 22) / 2
            readonly property real heightMm: model.height_mm || 25
            // Use model.z which is already computed in Python (height/2)
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                source: "#Cylinder"
                eulerRotation.x: 90
                // Qt Quick 3D #Cylinder: diameter=100, height=100
                scale: Qt.vector3d(parent.radiusMm / 50, parent.heightMm / 100, parent.radiusMm / 50)
                materials: PrincipledMaterial {
                    // Match legacy colors: OrangeBarrel=(1.0,0.5,0.063), BlueBarrel=(0.28,0.44,0.95)
                    baseColor: {
                        if (model.type === "barrel_orange")
                            return model.visible ? "#ff8010" : "#994d0a"
                        if (model.type === "barrel_blue")
                            return model.visible ? "#4770f2" : "#2a4391"
                        return model.visible ? "#18d0ff" : "#0e7e97"  // fallback cyan
                    }
                    roughness: 0.45
                    cullMode: Material.NoCulling
                    emissiveFactor: {
                        if (!model.visible)
                            return Qt.vector3d(0, 0, 0)
                        if (model.type === "barrel_orange")
                            return Qt.vector3d(0.15, 0.08, 0.01)  // warm orange glow
                        if (model.type === "barrel_blue")
                            return Qt.vector3d(0.05, 0.08, 0.18)  // cool blue glow
                        return Qt.vector3d(0.06, 0.2, 0.25)  // fallback cyan glow
                    }
                }
            }
        }
    }

    Component {
        id: markerDelegate
        Node {
            property var model
            visible: !model.missing
            // Legacy AprilTag: width=38mm (Y-axis), height=48mm (Z-axis), thickness=2mm (X-axis)
            readonly property real widthMm: model.width_mm || 38
            readonly property real heightMm: model.height_mm || 48
            readonly property real thicknessMm: model.thickness_mm || 2
            // Use model.z which is already computed in Python (height/2 = 24mm)
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            // Main marker body
            Model {
                source: "#Cube"
                // Qt Quick 3D #Cube: edge length=100
                // Scale: X=thickness, Y=width, Z=height
                scale: Qt.vector3d(thicknessMm / 100, widthMm / 100, heightMm / 100)
                materials: PrincipledMaterial {
                    // Legacy apriltag color: (0.5, 0.3, 0.9) = #804ce6 purple
                    baseColor: model.type === "apriltag" ? "#804ce6" : "#202020"
                    emissiveFactor: model.visible
                                    ? (model.type === "apriltag"
                                       ? Qt.vector3d(0.08, 0.05, 0.15)  // purple glow
                                       : Qt.vector3d(0.1, 0.1, 0.1))
                                    : Qt.vector3d(0.02, 0.02, 0.02)
                    roughness: 0.2
                    cullMode: Material.NoCulling
                }
            }

            // Hidden 2D Image to load texture from image provider
            Image {
                id: tagImage
                source: model.type === "apriltag" ? "image://tagtexture/" + String(model.marker_id) : ""
                visible: false
                cache: true
            }
            
            // Text label panel - Front face (+X direction)
            Model {
                id: textPanelFront
                visible: model.type === "apriltag" && model.marker_id !== null && model.marker_id !== undefined && model.marker_id !== ""
                source: "#Cube"
                // Very thin panel; after X-rotation by 90°, original Y→Z, Z→Y in world coords
                // So we swap width/height in scale to match world coordinates
                scale: Qt.vector3d(0.005, heightMm / 100, widthMm / 100)
                // Position in front of the marker
                position: Qt.vector3d(thicknessMm / 2 + 0.5, 0, 0)
                // Correct rotation: Only X-axis 90 degrees
                eulerRotation: Qt.vector3d(90, 0, 0)
                
                materials: PrincipledMaterial {
                    baseColor: "#ffffff"
                    baseColorMap: Texture {
                        sourceItem: tagImage
                    }
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.06, 0.2) : Qt.vector3d(0, 0, 0)
                    roughness: 0.1
                    cullMode: Material.NoCulling
                }
            }
            
            // Text label panel - Back face (-X direction)
            Model {
                id: textPanelBack
                visible: model.type === "apriltag" && model.marker_id !== null && model.marker_id !== undefined && model.marker_id !== ""
                source: "#Cube"
                // Very thin panel; after X-rotation by 90°, original Y→Z, Z→Y in world coords
                // So we swap width/height in scale to match world coordinates
                scale: Qt.vector3d(0.005, heightMm / 100, widthMm / 100)
                // Position behind the marker
                position: Qt.vector3d(-thicknessMm / 2 - 0.5, 0, 0)
                // Same rotation as front (mirrored by position, not rotation)
                eulerRotation: Qt.vector3d(90, 0, 0)
                
                materials: PrincipledMaterial {
                    baseColor: "#ffffff"
                    baseColorMap: Texture {
                        sourceItem: tagImage
                    }
                    emissiveFactor: model.visible ? Qt.vector3d(0.1, 0.06, 0.2) : Qt.vector3d(0, 0, 0)
                    roughness: 0.1
                    cullMode: Material.NoCulling
                }
            }
        }
    }

    Component {
        id: cliffPointDelegate
        Node {
            property var model
            visible: !model.missing
            readonly property real radiusMm: (model.diameter_mm || 6) / 2
            // Pearl sits on ground (z already adjusted in Python)
            position: Qt.vector3d(model.x, model.y, model.z)

            Model {
                source: "#Sphere"
                // Qt Quick 3D #Sphere: diameter=100. Scale to mm diameter.
                scale: Qt.vector3d(parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100, parent.radiusMm * 2 / 100)
                materials: PrincipledMaterial {
                    // Bright red glowing pearls for cliff edges
                    baseColor: "#ff0000"  // Pure red
                    roughness: 0.2  // Shiny like a pearl
                    metalness: 0.0
                    cullMode: Material.NoCulling
                    emissiveFactor: Qt.vector3d(0.4, 0.0, 0.0)  // Strong red glow
                }
            }
        }
    }

    Component {
        id: wallDelegate
        Node {
            property var model
            visible: !model.missing
            readonly property real lengthMm: model.length_mm || 300
            readonly property real heightMm: model.height_mm || 210
            readonly property real thicknessMm: model.thickness_mm || 4
            // Use model.z which is already computed in Python (height/2)
            position: Qt.vector3d(model.x, model.y, model.z)
            eulerRotation.z: radiansToDegrees(model.theta)

            Model {
                source: "#Cube"
                // Qt Quick 3D #Cube: edge length=100
                scale: Qt.vector3d(thicknessMm / 100, lengthMm / 100, heightMm / 100)
                materials: PrincipledMaterial {
                    // Cliff edges use bright red, regular walls use gray
                    baseColor: {
                        if (model.id && String(model.id).indexOf("cliff_") === 0) {
                            return "#ff0000"  // Bright red for cliff edges
                        }
                        return model.visible ? "#777777" : "#444444"
                    }
                    roughness: 0.4  // Slightly shiny for cliff visibility
                    cullMode: Material.NoCulling
                    emissiveFactor: {
                        if (model.id && String(model.id).indexOf("cliff_") === 0) {
                            return Qt.vector3d(0.3, 0.0, 0.0)  // Red glow for cliff edges
                        }
                        return model.visible ? Qt.vector3d(0.05, 0.05, 0.05) : Qt.vector3d(0, 0, 0)
                    }
                }
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
                case "wall":
                    return wallDelegate
                case "cliff_point":
                    return cliffPointDelegate
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
                if (!component) {
                    console.warn("No component for type:", data.type, "id:", data.id)
                    return
                }

                delegateItem = component.createObject(worldDelegateRoot, {
                    "model": data
                })
                if (!delegateItem) {
                    console.warn("Failed to create delegate for type", data.type, "id:", data.id)
                }
                // Debug cliff wall creation (comment out to reduce spam)
                // else if (data.id && String(data.id).indexOf("cliff_") === 0) {
                //     console.log("Created cliff wall:", data.id, "at (", data.x, ",", data.y, ",", data.z, ")",
                //                "size:", data.length_mm, "x", data.thickness_mm, "x", data.height_mm)
                // }
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
