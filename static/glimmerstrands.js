var Line = /** @class */ (function () {
    function Line(data, style) {
        if (style === void 0) { style = ""; }
        this.style = style;
        this.data = data;
        var min = Number.MAX_VALUE;
        var max = Number.MIN_VALUE;
        data.forEach(function (num) {
            if (num < min) {
                min = num;
            }
            if (num > max) {
                max = num;
            }
        });
        this.min = min;
        this.max = max;
    }
    Line.prototype.addPoint = function (point) {
        if (point < this.min) {
            this.min = point;
        }
        if (point > this.max) {
            this.max = point;
        }
        this.data.push(point);
    };
    return Line;
}());
var GlimmerStrands = /** @class */ (function () {
    function GlimmerStrands(container) {
        this.lines = [];
        this.el = document.createElementNS("http://www.w3.org/2000/svg", "svg");
        this.el.setAttribute('width', "100%");
        this.el.setAttribute('height', "100%");
        this.el.setAttribute('viewBox', "0 -0.1 1 1.1");
        this.el.setAttribute('preserveAspectRatio', "none");
        container.appendChild(this.el);
    }
    GlimmerStrands.prototype.addLine = function (line) {
        this.lines.push(line);
    };
    GlimmerStrands.prototype.addData = function (data) {
        var _this = this;
        data.forEach(function (point, index) {
            _this.lines[index].addPoint(point);
        });
    };
    GlimmerStrands.prototype.clear = function () {
        while (this.el.firstChild) {
            this.el.firstChild.remove();
        }
    };
    GlimmerStrands.prototype.render = function () {
        var _this = this;
        var min = Number.MAX_VALUE;
        var max = Number.MIN_VALUE;
        this.lines.forEach(function (line) {
            if (line.min < min) {
                min = line.min;
            }
            if (line.max > max) {
                max = line.max;
            }
        });
        this.clear();
        this.lines.forEach(function (line) {
            if (line.data.length <= 1) {
                return;
            }
            var polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
            polyline.setAttribute('vector-effect', "non-scaling-stroke");
            var points = [];
            line.data.forEach(function (dataPoint, index) {
                var x = index * 1.0 / (line.data.length - 1);
                var y = 1.0 - dataPoint / max;
                points.push(x.toString() + "," + y.toString());
            });
            polyline.setAttribute("points", points.join(" "));
            polyline.setAttribute("style", line.style);
            _this.el.appendChild(polyline);
        });
    };
    return GlimmerStrands;
}());
