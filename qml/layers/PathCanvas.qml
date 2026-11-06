import QtQuick 2.15

Canvas {
    id: canvas

    property var viewState
    property var nodeModel
    property var edgeModel
    property var overlayModel
    property var obstacleModel
    property var robotModel

    property color backgroundColor: "#111820"
    property color gridColor: "#1d2a35"
    property int gridSpacingMm: 200

    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    Connections {
        target: viewState
        enabled: !!viewState
        ignoreUnknownSignals: true
        function onChanged() { canvas.requestPaint(); }
    }

    Connections {
        target: nodeModel
        enabled: !!nodeModel
        ignoreUnknownSignals: true
        function onRevisionChanged() { canvas.requestPaint(); }
    }

    Connections {
        target: edgeModel
        enabled: !!edgeModel
        ignoreUnknownSignals: true
        function onRevisionChanged() { canvas.requestPaint(); }
    }

    Connections {
        target: overlayModel
        enabled: !!overlayModel
        ignoreUnknownSignals: true
        function onRevisionChanged() { canvas.requestPaint(); }
    }

    Connections {
        target: obstacleModel
        enabled: !!obstacleModel
        ignoreUnknownSignals: true
        function onRevisionChanged() { canvas.requestPaint(); }
    }

    Connections {
        target: robotModel
        enabled: !!robotModel
        ignoreUnknownSignals: true
        function onRevisionChanged() { canvas.requestPaint(); }
    }

    function pixelsPerMm() {
        if (!viewState || viewState.zoom === undefined || viewState.zoom === null)
            return 0.4;
        return Math.max(0.01, viewState.zoom);
    }

    function centerX() {
        if (!viewState || viewState.centerX === undefined)
            return 0;
        return viewState.centerX;
    }

    function centerY() {
        if (!viewState || viewState.centerY === undefined)
            return 0;
        return viewState.centerY;
    }

    function mapPoint(x, y) {
        var scale = pixelsPerMm();
        return {
            x: (centerY() - y) * scale + width / 2,
            y: (centerX() - x) * scale + height / 2
        };
    }

    function colorString(color, fallbackAlpha) {
        if (!color || color.length < 3)
            return "rgba(255,255,255," + fallbackAlpha + ")";
        var a = color.length >= 4 ? color[3] : fallbackAlpha;
        var r = Math.round(Math.max(0, Math.min(1, color[0])) * 255);
        var g = Math.round(Math.max(0, Math.min(1, color[1])) * 255);
        var b = Math.round(Math.max(0, Math.min(1, color[2])) * 255);
        return "rgba(" + r + "," + g + "," + b + "," + Math.max(0, Math.min(1, a)) + ")";
    }

    function drawGrid(ctx) {
        var scale = pixelsPerMm();
        if (scale <= 0.0)
            return;
        var spacing = Math.max(50, gridSpacingMm);
        var spanX = width / scale;
        var spanY = height / scale;
        var minX = centerX() - spanX / 2;
        var maxX = centerX() + spanX / 2;
        var minY = centerY() - spanY / 2;
        var maxY = centerY() + spanY / 2;
        var startX = Math.floor(minX / spacing) * spacing;
        var startY = Math.floor(minY / spacing) * spacing;

        ctx.save();
        ctx.strokeStyle = gridColor;
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (var gx = startX; gx <= maxX; gx += spacing) {
            var a = mapPoint(gx, minY);
            var b = mapPoint(gx, maxY);
            ctx.moveTo(a.x + 0.5, a.y + 0.5);
            ctx.lineTo(b.x + 0.5, b.y + 0.5);
        }
        for (var gy = startY; gy <= maxY; gy += spacing) {
            var c = mapPoint(minX, gy);
            var d = mapPoint(maxX, gy);
            ctx.moveTo(c.x + 0.5, c.y + 0.5);
            ctx.lineTo(d.x + 0.5, d.y + 0.5);
        }
        ctx.stroke();
        ctx.restore();
    }

    function drawMarkers(ctx) {
        if (!nodeModel)
            return;
        var count = nodeModel.count || 0;
        var scale = pixelsPerMm();
        for (var i = 0; i < count; ++i) {
            var entry = nodeModel.get(i);
            if (!entry)
                continue;
            var halfSize = (entry.size || 4.0) * 0.5 * scale;
            var center = mapPoint(entry.x, entry.y);
            ctx.save();
            ctx.translate(center.x, center.y);
            ctx.beginPath();
            ctx.rect(-halfSize, -halfSize, halfSize * 2, halfSize * 2);
            ctx.fillStyle = colorString(entry.color, 1.0);
            ctx.fill();
            ctx.restore();
        }
    }

    function drawPolylines(ctx, model) {
        if (!model)
            return;
        var count = model.count || 0;
        var scale = pixelsPerMm();
        for (var i = 0; i < count; ++i) {
            var entry = model.get(i);
            if (!entry || !entry.points || entry.points.length < 4)
                continue;
            ctx.save();
            ctx.beginPath();
            var mapped = mapPoint(entry.points[0], entry.points[1]);
            ctx.moveTo(mapped.x, mapped.y);
            for (var p = 2; p < entry.points.length; p += 2) {
                mapped = mapPoint(entry.points[p], entry.points[p + 1]);
                ctx.lineTo(mapped.x, mapped.y);
            }
            ctx.strokeStyle = colorString(entry.color, 1.0);
            ctx.lineWidth = Math.max(1, (entry.width || 1.0) * scale);
            ctx.stroke();
            ctx.restore();
        }
    }

    function drawCircle(ctx, item) {
        var center = mapPoint(item.x, item.y);
        var scale = pixelsPerMm();
        var radius = (item.radius || 0) * scale;
        ctx.save();
        ctx.beginPath();
        ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
        if (item.filled) {
            ctx.fillStyle = colorString(item.color, 0.5);
            ctx.fill();
        } else {
            ctx.strokeStyle = colorString(item.color, 1.0);
            ctx.lineWidth = Math.max(1, 1.2 * scale);
            ctx.stroke();
        }
        ctx.restore();
    }

    function drawRectangle(ctx, item) {
        var center = mapPoint(item.x, item.y);
        var scale = pixelsPerMm();
        var widthPx = (item.width || 0) * scale;
        var heightPx = (item.height || 0) * scale;
        ctx.save();
        ctx.translate(center.x, center.y);
        ctx.rotate(-((item.rotation || 0) + 90) * Math.PI / 180);
        if (item.filled) {
            ctx.fillStyle = colorString(item.color, 0.5);
            ctx.fillRect(-widthPx / 2, -heightPx / 2, widthPx, heightPx);
        } else {
            ctx.strokeStyle = colorString(item.color, 1.0);
            ctx.lineWidth = Math.max(1, 1.2 * scale);
            ctx.strokeRect(-widthPx / 2, -heightPx / 2, widthPx, heightPx);
        }
        ctx.restore();
    }

    function drawShapes(ctx, model) {
        if (!model)
            return;
        var count = model.count || 0;
        for (var i = 0; i < count; ++i) {
            var item = model.get(i);
            if (!item || !item.type)
                continue;
            if (item.type === "circle")
                drawCircle(ctx, item);
            else if (item.type === "rectangle")
                drawRectangle(ctx, item);
        }
    }

    onPaint: {
        var ctx = getContext("2d");
        ctx.reset();
        ctx.fillStyle = backgroundColor;
        ctx.fillRect(0, 0, width, height);

        drawGrid(ctx);
        drawShapes(ctx, obstacleModel);
        drawPolylines(ctx, edgeModel);
        drawPolylines(ctx, overlayModel);
        drawShapes(ctx, robotModel);
        drawMarkers(ctx);
    }
}
