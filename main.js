
var a3d = a3d || {};

a3d.Program = function(maxX, maxY, maxFlex){
  this.maxX_ = maxX;
  this.maxY_ = maxY;
  this.maxFlex_ = maxFlex;
};


a3d.Program.prototype.parse = function(code) {
  this.codes_ = [];
  this.xyCodes_ = [];
  this.layerStarts_ = [];
  this.l_ = {
    x: 0,
    y: 0
  };
  this.u_ = {
    x: 0,
    y: 0
  };
  this.index_ = 0;
  this.parse_(code);
  this.calculateABaxis();
  this.buildOutput_();
};

a3d.Program.prototype.getMaxX = function() {
  return this.maxX_;
};

a3d.Program.prototype.getMaxY = function() {
  return this.maxY_;
};

a3d.Program.prototype.getMaxFlex = function() {
  return this.maxFlex_;
};

a3d.Program.prototype.getNumberOfLayers = function() {
  return this.layerStarts_.length;
};

a3d.Program.prototype.getLayer = function(layerNumber) {
  var start = this.layerStarts_[layerNumber - 1];
  var end = Math.min(this.layerStarts_[layerNumber], this.codes_.length);
  end = end || this.codes_.length;
  var moves = [];
  for (var i = start; i < end; i++) {
    var code = this.codes_[i];
    if (typeof code != "string") {
      if (code.x != null) {
        moves.push( {
          x: code.x,
          y: code.y,
          a: code.a,
          b: code.b
        })
      }
    }
  }

  var prependCode = null;
  for (i = start; i >= 0; i--) {
    code = this.codes_[i];
    if (typeof code != "string") {
      if (code.x != null) {
        prependCode = {
          x: code.x,
          y: code.y,
          a: code.a,
          b: code.b
        };
        break;
      }
    }
  }
  if (prependCode != null) {
    moves.unshift(prependCode);
  } else {
    moves.unshift({
      x: 0,
      y: 0,
      a: 0,
      b: 0
    })
  }

  return moves;
};

a3d.Program.prototype.buildOutput_ = function() {
  var s = "";
  for (var i = 0; i < this.codes_.length; i++) {
    var code = this.codes_[i];
    if (typeof code != "string") {
      var line = "G01"
        + (code.x == null ? "" : " X" + code.x.toFixed(3))
        + (code.y == null ? "" : " Y" + code.y.toFixed(3))
        + (code.z == null ? "" : " Z" + code.z.toFixed(3))
        + (code.a == null ? "" : " A" + code.a.toFixed(3))
        + (code.b == null ? "" : " B" + code.b.toFixed(3))
        + (code.c == null ? "" : " C" + code.c.toFixed(4))
        + (code.f == null ? "" : " F" + code.f.toFixed(0));
      s = s.concat(line, "\n")
    } else {
      s = s.concat(code, "\n")
    }
  }
  this.output_ = s;
};

a3d.Program.prototype.getOutput = function() {
  return this.output_;
};

a3d.Program.prototype.calculateABaxis = function() {
  while (this.index_ < this.xyCodes_.length) {
    this.step();
    this.index_++;
  }
};

a3d.Program.prototype.step = function() {
  // l0 and u0 are where the lower and upper gantry are right now
  // l1 is the current step (and next position of lower gantry)

  var l1 = this.xyCodes_[this.index_];
  if (l1.x > this.maxX_) {
    this.maxX_ = l1.x;
  }
  if (l1.y > this.maxY_) {
    this.maxY_ = l1.y;
  }
  if (l1.x > this.maxY_) {
    this.maxY_ = l1.x;
  }
  if (l1.y > this.maxX_) {
    this.maxX_ = l1.y;
  }


  var dU0L1 = geo.distance(this.u_, l1);

  // Type 5 - Last step in the program
  if (this.index_ + 1 == this.xyCodes_.length) {
    this.stepType5_(l1, dU0L1);
  } else {
    // Type 1 or 2
    if (dU0L1 > this.maxFlex_) {
      this.stepType1or2_(l1);
    } else {
      this.stepType3or4_(l1);
    }
  }
  this.u_ = l1.getU();
  this.l_ = l1.getL();
};

a3d.Program.prototype.stepType1or2_ = function(l1) {
  // type 1 or type 2
  var l2 = this.xyCodes_[this.index_ + 1];
  var dL1L2 = geo.distance(l1, l2);
  if (dL1L2 > this.maxFlex_) {
    this.stepType1_(l1, l2)
  } else {
    this.stepType2_(l1, l2, dL1L2)
  }
};

a3d.Program.prototype.stepType1_ = function(l1, l2) {
  var u1 = geo.cutTheCorner(l1, this.l_, l2, this.maxFlex_);
 l1.setU(u1);
};

a3d.Program.prototype.stepType2_ = function(l1, l2, dL1L2) {
  var u1 = geo.cutTheCorner(l1, this.l_, l2, dL1L2 / 2);
  l1.setU(u1);
};

a3d.Program.prototype.stepType3or4_ = function(l1) {
  var m1Response = this.findM1_();
  if (m1Response == null) {
    // there are no future steps farther than the
    // max flex distance. Stay where we are.
    l1.setU(this.u_);
  } else {
    var dL0L1 = geo.distance(this.l_, l1);
    if (m1Response.dLm0Lm1 <= this.maxFlex_) {
      //type3
      this.stepType3_(l1, m1Response, dL0L1);
    } else {
      //type4
      this.stepType4_(l1, m1Response, dL0L1);
    }

  }
};

a3d.Program.prototype.stepType3_ = function(l1, m1Response, dL0L1) {
  var um0 = geo.midpoint(m1Response.lm0, m1Response.lm1);
  var u1 = geo.scaledPointOnLine(this.u_, um0, dL0L1 / m1Response.pathLength);
  l1.setU(u1);
};

a3d.Program.prototype.stepType4_ = function(l1, m1Response, dL0L1) {
  var lmNeg1 = this.xyCodes_[m1Response.m1 - 2];
  var um0 = geo.cutTheCorner(m1Response.lm0, lmNeg1, m1Response.lm1, this.maxFlex_);
  var u1 = geo.scaledPointOnLine(this.u_, um0, dL0L1 / m1Response.pathLength);
  l1.setU(u1);
};

a3d.Program.prototype.findM1_ = function() {
  var pathLength = 0;
  for (var m1 = this.index_; m1 < this.xyCodes_.length; m1++) {

    var lm0 = m1 == this.index_ ? this.l_ : this.xyCodes_[m1 - 1];
    var lm1 = this.xyCodes_[m1];

    var dLm0Lm1 = geo.distance(lm0, lm1);
    var dU0Lm1 = geo.distance(this.u_, lm1);

    if (dU0Lm1 > this.maxFlex_) {
      return {
        'lm1': lm1,
        'lm0': lm0,
        'm1': m1,
        'pathLength': pathLength,
        'dLm0Lm1': dLm0Lm1
      }
    }
    pathLength += dLm0Lm1;
  }
  return null;
};

a3d.Program.prototype.stepType5_ = function(l1, dU0L1) {
  if (dU0L1 < this.maxFlex_) {
    // if it is within the max flex distance,
    // then we don't move U
    l1.setU(this.u_);
  } else {
    var u0 = geo.scaledPointOnLine(this.u_, l1, (dU0L1 - this.maxFlex_) / dU0L1);
    l1.setU(u0);
  }
};



a3d.Program.prototype.parse_ = function(code) {
  var lines = code.split("\n");
  for (var i = 0; i < lines.length; i++) {
    var oldLine = lines[i].trim();
    var line = oldLine.toUpperCase();
    if (line.startsWith("G")) {
      this.parseGCode_(line);
    } else {
      if (oldLine.startsWith("; layer")) {
        this.layerStarts_.push(this.codes_.length)
      }
      this.parseOtherLine_(oldLine);
    }
  }
};

a3d.Program.prototype.parseGCode_ = function(line) {
  if (line.startsWith("G1") || line.startsWith("G01")) {
    this.parseG01Code_(line)
  } else {
    this.parseOtherLine_(line)
  }
};

a3d.Program.prototype.parseG01Code_ = function(line) {
  var parts = line.split(" ");
  var g01 = new a3d.Program.G01();
  for (var i = 0; i < parts.length; i++) {
    var part = parts[i];
    if (part.length > 0) {
      switch(part.charAt(0)) {
        case "X":
          try {
            g01.x = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "Y":
          try {
            g01.y = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "Z":
          try {
            g01.z = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "A":
          try {
            g01.a = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "B":
          try {
            g01.b = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "C":
          try {
            g01.c = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
        case "F":
          try {
            g01.f = parseFloat(part.slice(1));
          }
          catch(e){}
          break;
      }
    }
  }
  if (g01.isXY()) {
    this.xyCodes_.push(g01);
  }
  // if (g01.isZ()) {
  //   this.layerStarts_.push(this.codes_.length)
  // }
  this.codes_.push(g01);
};

a3d.Program.prototype.parseOtherLine_ = function(line) {
  this.codes_.push(line)
};

a3d.Program.G01 = function() {
  this.x = null;
  this.y = null;
  this.z = null;
  this.a = null;
  this.b = null;
  this.c = null;
  this.f = null;
};

a3d.Program.G01.prototype.isXY = function() {
  return this.x != null || this.y != null;
};

a3d.Program.G01.prototype.isZ = function() {
  return this.z != null;
};

a3d.Program.G01.prototype.setU = function(p) {
  this.a = p.x;
  this.b = p.y;
};

a3d.Program.G01.prototype.setL = function(p) {
  this.x = p.x;
  this.y = p.y;
};

a3d.Program.G01.prototype.getU = function() {
  return {
    x: this.a,
    y: this.b
  }
};

a3d.Program.G01.prototype.getL = function() {
  return {
    x: this.x,
    y: this.y
  }
};

var geo = geo || {};

geo.distance = function(p0, p1) {
  return Math.sqrt(Math.pow(p1.x-p0.x, 2) + Math.pow(p1.y-p0.y, 2))
};

geo.midpoint = function(p0, p1) {
  return geo.scaledPointOnLine(p0, p1, .5)
};

geo.scaledPointOnLine = function(p0, p1, s) {
  var x = p0.x + (p1.x - p0.x) * s;
  var y = p0.y + (p1.y - p0.y) * s;
  return {'x': x, 'y': y}
};

geo.cutTheCorner = function(originPoint, point1, point2, distance) {
  var p1 = {
    p: "p1",
    x: point1.x - originPoint.x,
    y: point1.y - originPoint.y
  };
  var p2 = {
    p: "p2",
    x: point2.x - originPoint.x,
    y: point2.y - originPoint.y
  };
  var a1 = geo.getAngle(p1);
  var a2 = geo.getAngle(p2);
  var lAngle = a1 > a2 ? a1 : a2;
  var sAngle = a1 > a2 ? a2 : a1;
  var bisectedAngle = (lAngle - sAngle)/2 + sAngle;
  var a = (bisectedAngle - sAngle) >= Math.PI / 2 ? bisectedAngle : bisectedAngle - Math.PI;
  var p = {
    x: originPoint.x - distance * Math.cos(a),
    y: originPoint.y - distance * Math.sin(a)
  };
  return p;
};

geo.getAngle = function(p) {
  var a = Math.atan2(p.y, p.x);
  return a < 0 ? a + 2 * Math.PI : a;
};



a3d.Main = {};

a3d.Main.run = function(e) {
  var inputCode = a3d.Main.inputBox.value;
  var maxX = parseInt(a3d.Main.maxXInput.value);
  var maxY = parseInt(a3d.Main.maxYInput.value);
  var maxFlex = parseInt(a3d.Main.maxFlexInput.value);


  a3d.Main.program = new a3d.Program(maxX, maxY, maxFlex);
  a3d.Main.program.parse(inputCode);

  a3d.Main.outputBox.value = a3d.Main.program.getOutput();
  a3d.Main.maxXInput.value = a3d.Main.program.getMaxX();
  a3d.Main.maxYInput.value = a3d.Main.program.getMaxY();


  var numOfLayers = a3d.Main.program.getNumberOfLayers();
  if (numOfLayers > 0) {
    a3d.Main.layerInput.value = "1";
    a3d.Main.layerInput.max = "" + numOfLayers;
    a3d.Main.layerInput.min = "1";
    a3d.Main.display(null);
  }
};

a3d.Main.display = function(e) {
  var layerNumber = parseInt(a3d.Main.layerInput.value);
  var moves = a3d.Main.program.getLayer(layerNumber);
  var maxX = a3d.Main.program.getMaxX();
  var maxY = a3d.Main.program.getMaxY();
  var maxFlex = a3d.Main.program.getMaxFlex();
  if (a3d.Main.canvas.getContext) {
    var ctx = a3d.Main.canvas.getContext('2d');
    var cH = ctx.canvas.height;
    var cW = ctx.canvas.width;
    ctx.clearRect(0, 0, cW, cH);


    ctx.beginPath();
    ctx.strokeStyle = "#008080";
    for (var i = 0; i < moves.length; i++) {
      var move = moves[i];
      var x = cW * move.x / maxX;
      var y = cH * move.y / maxY;
      if (i == 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();

    // ctx.strokeStyle = "#5bcc29";
    // for ( i = 0; i < moves.length; i++) {
    //   move = moves[i];
    //   x = cW * move.x / maxX;
    //   y = cH * move.y / maxY;
    //   ctx.beginPath();
    //   ctx.arc(x, y, cW * maxFlex / maxX, 0, Math.PI * 2);
    //   ctx.stroke();
    // }


    ctx.beginPath();
    ctx.strokeStyle = "#804876";
    for (i = 0; i < moves.length; i++) {
      move = moves[i];
      var a = cW * move.a / maxX;
      var b = cH * move.b / maxY;
      if (i == 0) {
        ctx.moveTo(a, b);
      } else {
        ctx.lineTo(a, b);
      }
    }
    ctx.stroke();
  }
  a3d.Main.layerLabel.innerText = "Showing layer " + layerNumber + " out of " + a3d.Main.program.getNumberOfLayers();
};

a3d.Main.canvas = document.getElementById("canvas");
a3d.Main.inputBox = document.getElementById("input");
a3d.Main.outputBox = document.getElementById("output");
a3d.Main.runButton = document.getElementById("run");
a3d.Main.displayButton = document.getElementById("display");
a3d.Main.maxFlexInput = document.getElementById("maxFlex");
a3d.Main.maxXInput = document.getElementById("maxX");
a3d.Main.maxYInput = document.getElementById("maxY");
a3d.Main.layerInput = document.getElementById("layer");
a3d.Main.layerLabel = document.getElementById("layerLabel");

a3d.Main.runButton.addEventListener("click", a3d.Main.run);
a3d.Main.displayButton.addEventListener("click", a3d.Main.display);


a3d.Main.angle1 = 0;
a3d.Main.angle2 = 0;
a3d.Main.origin = {x: 320, y: 320};

var ctx = document.getElementById("canvas").getContext('2d');
ctx.id = "tracking id";

var COLOR_TEAL = "#008080";

var COLOR_GREEN = "#5bcc29";

var COLOR_PURPLE = "#804876";

var run = true;
a3d.Main.testAngle = function() {
  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  var a1 = a3d.Main.angle1 > 360 ? a3d.Main.angle1 - 360 : a3d.Main.angle1;
  var a2 = a3d.Main.angle2 > 360 ? a3d.Main.angle2 - 360 : a3d.Main.angle2;
  var p0 = a3d.Main.origin;
  var p1 = {
    x: p0.x - 200 * Math.cos(Math.PI * a1 / 180),
    y: p0.y - 200 * Math.sin(Math.PI * a1 / 180)
  };
  var p2 = {
    x: p0.x - 200 * Math.cos(Math.PI * a2 / 180),
    y: p0.y - 200 * Math.sin(Math.PI * a2 / 180)
  };
  var p3 = geo.cutTheCorner(p0, p1, p2, 100);

  a3d.Main.drawLine(p0, p1, COLOR_TEAL);
  a3d.Main.drawLine(p0, p2, COLOR_TEAL);
  a3d.Main.drawLine(p0, p3, COLOR_PURPLE);

  ctx.strokeStyle = COLOR_GREEN;
  ctx.beginPath();
  ctx.arc(p0.x, p0.y, 100, 0, Math.PI * 2);
  ctx.stroke();

  a3d.Main.angle1 += 15;
  if (a3d.Main.angle1 > a3d.Main.angle2 + 360) {
    a3d.Main.angle2 += 15;
    a3d.Main.angle1 = a3d.Main.angle2;
  }
  run=false;

};

a3d.Main.drawLine = function(p0, p1, color) {

  ctx.strokeStyle = color;
  ctx.beginPath();
  ctx.moveTo(p0.x, p0.y);
  ctx.lineTo(p1.x, p1.y);
  ctx.stroke();


};


//a3d.Main.displayButton.addEventListener("click", a3d.Main.testAngle);






