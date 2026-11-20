/* LOCH Timer · figuras vectoriales sin rostro.
   Cada ejercicio = 2 poses (A y B) que se interpolan. Las poses se escriben con posiciones
   (x hacia adelante, h = altura sobre el piso) y un solver de 2 huesos calcula codos y rodillas,
   así las extremidades nunca cambian de largo. */
(function (root) {
  'use strict';

  var TOR = 42, UA = 25, FA = 24, TH = 34, SH = 32, FT = 13, NECK = 7, R = 4;
  var BODY = {
    m: { limb: 8.5, torso: 15.5, sw: 16, hw: 9, sr: 8.5, hr: 7, head: 9 },
    f: { limb: 7.5, torso: 12, sw: 13, hw: 10.5, sr: 6.5, hr: 8.5, head: 8.5 }
  };

  /* ---------- Matemática ---------- */
  function rad(d) { return d * Math.PI / 180; }
  function dirv(t) { return [Math.sin(rad(t)), -Math.cos(rad(t))]; }
  function mix(a, b, s) {
    if (typeof a === 'number' && typeof b === 'number') return a + (b - a) * s;
    if (Array.isArray(a) && Array.isArray(b)) return a.map(function (v, i) { return mix(v, b[i], s); });
    if (a && b && typeof a === 'object') {
      var o = {}, k;
      for (k in a) o[k] = (k in b) ? mix(a[k], b[k], s) : a[k];
      for (k in b) if (!(k in a)) o[k] = b[k];
      return o;
    }
    return a === undefined ? b : a;
  }
  function P() {
    var o = {};
    for (var i = 0; i < arguments.length; i++) { var a = arguments[i]; for (var k in a) if (a[k] !== undefined) o[k] = a[k]; }
    return o;
  }
  // Solver de 2 huesos. hint = hacia dónde "apunta" el codo/rodilla.
  function ik(r, t, l1, l2, hint) {
    var dx = t[0] - r[0], dy = t[1] - r[1], d = Math.hypot(dx, dy), short = 0;
    if (d < 0.001) { dx = 0; dy = -0.001; d = 0.001; }
    var max = l1 + l2 - 0.02, min = Math.abs(l1 - l2) + 0.5;
    if (d > max) { short = d - max; var k = max / d; dx *= k; dy *= k; d = max; }
    else if (d < min) { var k2 = min / d; dx *= k2; dy *= k2; d = min; }
    var a = (l1 * l1 - l2 * l2 + d * d) / (2 * d);
    var h = Math.sqrt(Math.max(0, l1 * l1 - a * a));
    var ux = dx / d, uy = dy / d, px = -uy, py = ux;
    var side = (px * hint[0] + py * hint[1]) >= 0 ? 1 : -1;
    return { mid: [r[0] + ux * a + px * h * side, r[1] + uy * a + py * h * side], end: [r[0] + dx, r[1] + dy], short: short };
  }

  /* ---------- Esqueleto ---------- */
  function solve(Pz, view, sex) {
    var B = BODY[sex], front = view === 'front';
    var hip = Pz.hip, sh;
    if (Pz.sh) {
      var dx = Pz.sh[0] - hip[0], dy = Pz.sh[1] - hip[1], d = Math.hypot(dx, dy) || 1;
      sh = [hip[0] + dx / d * TOR, hip[1] + dy / d * TOR];
    } else { var v = dirv(Pz.t); sh = [hip[0] + v[0] * TOR, hip[1] + v[1] * TOR]; }
    var tdeg = Math.atan2(sh[0] - hip[0], -(sh[1] - hip[1])) * 180 / Math.PI;
    var hu = dirv(tdeg + (Pz.hd || 0));
    var hc = [sh[0] + hu[0] * (NECK + B.head), sh[1] + hu[1] * (NECK + B.head)];
    var aN = front ? [sh[0] + B.sw, sh[1]] : sh, aF = front ? [sh[0] - B.sw, sh[1]] : sh;
    var lN = front ? [hip[0] + B.hw, hip[1]] : hip, lF = front ? [hip[0] - B.hw, hip[1]] : hip;
    function mir(v) { return [-v[0], v[1]]; }
    function rel(r, o) { return [r[0] + o[0], r[1] + o[1]]; }
    var ekN = Pz.ek || [-1, 0], kkN = Pz.kk || [1, 0];
    var hN = Pz.hr ? rel(aN, Pz.hr) : Pz.hand;
    var hF = Pz.hr2 ? rel(aF, Pz.hr2) : Pz.hand2 ? Pz.hand2 : front ? (Pz.hr ? rel(aF, mir(Pz.hr)) : mir(Pz.hand)) : hN;
    var fN = Pz.foot, fF = Pz.foot2 || (front ? mir(Pz.foot) : Pz.foot);
    var ekF = Pz.ek2 || (front ? mir(ekN) : ekN), kkF = Pz.kk2 || (front ? mir(kkN) : kkN);
    var fdN = Pz.fd === undefined ? 90 : Pz.fd;
    var fdF = Pz.fd2 !== undefined ? Pz.fd2 : (front ? -fdN : fdN);
    var a1 = ik(aN, hN, UA, FA, ekN), a2 = ik(aF, hF, UA, FA, ekF);
    var l1 = ik(lN, fN, TH, SH, kkN), l2 = ik(lF, fF, TH, SH, kkF);
    function toe(an, fd) { var f = dirv(fd); return [an[0] + f[0] * FT, an[1] + f[1] * FT]; }
    return {
      view: view, B: B, hip: hip, sh: sh, head: hc, hu: hu, tdeg: tdeg,
      armN: [aN, a1.mid, a1.end], armF: [aF, a2.mid, a2.end],
      legN: [lN, l1.mid, l1.end, toe(l1.end, fdN)], legF: [lF, l2.mid, l2.end, toe(l2.end, fdF)],
      short: Math.max(a1.short, a2.short, l1.short, l2.short)
    };
  }

  /* ---------- Dibujo ---------- */
  function n1(v) { return Math.round(v * 10) / 10; }
  function pt(p) { return n1(p[0]) + ',' + n1(-p[1]); }
  function pl(pts, w, op) {
    return '<polyline points="' + pts.map(pt).join(' ') + '" fill="none" stroke="currentColor" stroke-width="' + w +
      '" stroke-linecap="round" stroke-linejoin="round"' + (op < 1 ? ' opacity="' + op + '"' : '') + '/>';
  }
  function circle(c, r, extra) { return '<circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="' + r + '" fill="currentColor"' + (extra || '') + '/>'; }
  var OUT = 'stroke="var(--bgc,#fff)" stroke-width="1.6"';

  function dumb(c, type) {
    if (type === 'o') return circle(c, 6.4, ' ' + OUT);
    var vert = type === 'v', ax = vert ? 0 : 9, ay = vert ? 9 : 0, px = vert ? 6 : 0, py = vert ? 0 : 6;
    var e1 = [c[0] - ax, c[1] - ay], e2 = [c[0] + ax, c[1] + ay], s = '';
    function ln(a, b, w, col) { return '<line x1="' + n1(a[0]) + '" y1="' + n1(-a[1]) + '" x2="' + n1(b[0]) + '" y2="' + n1(-b[1]) + '" stroke="' + col + '" stroke-width="' + w + '" stroke-linecap="round"/>'; }
    var bg = 'var(--bgc,#fff)', fg = 'currentColor';
    s += ln(e1, e2, 6.2, bg) + ln([e1[0] - px, e1[1] - py], [e1[0] + px, e1[1] + py], 8, bg) + ln([e2[0] - px, e2[1] - py], [e2[0] + px, e2[1] + py], 8, bg);
    s += ln(e1, e2, 3, fg) + ln([e1[0] - px, e1[1] - py], [e1[0] + px, e1[1] + py], 4.6, fg) + ln([e2[0] - px, e2[1] - py], [e2[0] + px, e2[1] + py], 4.6, fg);
    return s;
  }

  function chair(x0, x1, h, back) {
    var s = '<g stroke="currentColor" stroke-opacity=".42" stroke-width="3.6" stroke-linecap="round" fill="none">';
    s += '<line x1="' + x0 + '" y1="' + -h + '" x2="' + x1 + '" y2="' + -h + '"/>';
    s += '<line x1="' + (x0 + 3) + '" y1="' + -h + '" x2="' + (x0 + 3) + '" y2="-1"/>';
    s += '<line x1="' + (x1 - 3) + '" y1="' + -h + '" x2="' + (x1 - 3) + '" y2="-1"/>';
    if (back) s += '<line x1="' + back[0] + '" y1="' + -h + '" x2="' + back[0] + '" y2="' + -back[1] + '"/>';
    return s + '</g>';
  }

  function drawFigure(J, ex, sex, p, s) {
    var B = J.B, front = J.view === 'front', out = '';
    var u = J.hu, face = ex.face || 1, f = [u[1] * face, -u[0] * face], back = [-f[0], -f[1]];
    var op = front ? 1 : 0.5;
    if (ex.bg) out += ex.bg(J, p, s);
    function limb(c, w, o) { return pl(c, w, o); }
    // lejanas
    out += limb(J.legF, B.limb, op) + limb(J.armF, B.limb, op);
    if (ex.db && !ex.dbN) out += '<g opacity="' + (front ? 1 : 0.6) + '">' + dumb(J.armF[2], ex.db) + '</g>';
    // tronco
    if (front) {
      var sw = B.sw, hw = B.hw, S = J.sh, H = J.hip;
      out += '<path d="M' + pt([S[0] - sw + 2, S[1]]) + ' L' + pt([S[0] + sw - 2, S[1]]) + ' L' + pt([H[0] + hw - 1, H[1]]) + ' L' + pt([H[0] - hw + 1, H[1]]) + ' Z" fill="currentColor" stroke="currentColor" stroke-width="7" stroke-linejoin="round"/>';
    } else {
      out += pl([J.hip, J.sh], B.torso, 1);
      out += circle(J.hip, B.hr) + circle(J.sh, B.sr);
    }
    // cuello y cabeza
    var nk = [J.sh[0] + u[0] * 8, J.sh[1] + u[1] * 8];
    out += pl([J.sh, nk], B.limb * 0.75, 1);
    out += circle(J.head, B.head);
    if (sex === 'f') {
      if (front) {
        out += circle([J.head[0] + u[0] * (B.head + 1), J.head[1] + u[1] * (B.head + 1)], 4.6);
      } else {
        var gv = [back[0] * 0.5, back[1] * 0.5 - 1], gl = Math.hypot(gv[0], gv[1]) || 1;
        var b0 = [J.head[0] + back[0] * B.head * 0.6 + u[0] * 3, J.head[1] + back[1] * B.head * 0.6 + u[1] * 3];
        var b1 = [b0[0] + gv[0] / gl * 13, b0[1] + gv[1] / gl * 13];
        if (b1[1] < 3) b1[1] = 3;
        out += pl([b0, b1], 6, 1);
      }
    }
    // cercanas
    out += limb(J.legN, B.limb, 1) + limb(J.armN, B.limb, 1);
    if (ex.db) out += dumb(J.armN[2], ex.db);
    if (ex.fg) out += ex.fg(J, p, s);
    return out;
  }

  /* ---------- Poses ---------- */
  var STAND = { hip: [0, 66], t: 180, hd: 0, hand: [4, 60], foot: [0, R], fd: 90, ek: [-1, 0], kk: [1, 0.2] };
  var SUP = { hip: [0, R], sh: [-42, R], hd: -15, hand: [5, R], foot: [42, R], fd: 90, ek: [0, -1], kk: [0, 1] };
  var LEN = TOR + TH + SH;
  function lineSh(shH, an) { // cuerpo recto: calcula x del hombro dado el tobillo
    var sx = an[0] + Math.sqrt(LEN * LEN - Math.pow(shH - an[1], 2)), k = TOR / LEN, S = [sx, shH];
    return { sh: S, hip: [S[0] + (an[0] - S[0]) * k, S[1] + (an[1] - S[1]) * k], foot: an };
  }
  function lineAnk(S, ah) { // cuerpo recto: calcula x del tobillo dado el hombro
    var ax = S[0] - Math.sqrt(LEN * LEN - Math.pow(S[1] - ah, 2)), an = [ax, ah], k = TOR / LEN;
    return { sh: S, hip: [S[0] + (an[0] - S[0]) * k, S[1] + (an[1] - S[1]) * k], foot: an };
  }
  var HINGE = { hip: [-26, 60], t: 105, hd: 10, foot: [0, R], fd: 90, ek: [-1, 0.3], kk: [1, 0.2] };

  var pA = [-66, 15], pu0 = lineSh(51, pA), pu1 = lineSh(22, pA);
  var PLK = { hd: -12, fd: 20, ek: [-1, -1], kk: [0, 1] };
  var sp0 = lineAnk([30, 29], 8), sp1 = lineAnk([30, 29], 8);
  var fp0 = lineAnk([27, 29], 15), fp1 = lineAnk([27, 30], 15);
  var CL = { hip: [-4.6, 39.1], t: 105, hand: [36, R], hd: -12, fd: 20, ek: [-1, 0], kk: [1, 1], kk2: [1, 1] };

  var EX = [
    { id: 'arms', name: 'Giros de brazos', grp: 'Calentamiento', view: 'side', T: 1800, d: [60, 0, 1], tp: 0.25,
      fn: function (p) { var f = 2 * Math.PI * p; return P(STAND, { hand: [46 * Math.sin(f), 108 - 46 * Math.cos(f)], ek: [-1, 0] }); } },
    { id: 'jacks', name: 'Jumping jacks', grp: 'Calentamiento', view: 'front', T: 1000, d: [40, 15, 3],
      A: P(STAND, { hr: [8, -48], foot: [8, R], ek: [1, 0], kk: [1, 0] }),
      B: P(STAND, { hip: [0, 70], hr: [14, 44], foot: [27, 10], ek: [1, 0], kk: [1, 0] }) },

    { id: 'pushup', name: 'Flexiones', grp: 'Pecho y tríceps', view: 'side', T: 2000, d: [40, 45, 3],
      A: P(PLK, pu0, { hand: [pu0.sh[0], R] }), B: P(PLK, pu1, { hand: [pu0.sh[0], R] }) },
    { id: 'chestpress', name: 'Press de pecho en el piso', grp: 'Pecho y tríceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o',
      A: P(SUP, { hand: [-17, 28], ek: [0, -1] }), B: P(SUP, { hand: [-42, 51], ek: [1, 0] }) },
    { id: 'flies', name: 'Aperturas en el piso', grp: 'Pecho y tríceps', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(SUP, { hand: [-42, 50], ek: [1, 0.2] }), B: P(SUP, { hand: [2, 8], ek: [0, 1] }) },
    { id: 'dips', name: 'Fondos de tríceps en silla', grp: 'Pecho y tríceps', view: 'side', T: 2200, d: [40, 45, 3],
      A: P(STAND, { hip: [-12, 40], sh: [-24, 80], hand: [-26, 34], foot: [38, R], ek: [-1, 0], kk: [1, 1] }),
      B: P(STAND, { hip: [-10, 16], sh: [-22, 56], hand: [-26, 34], foot: [38, R], ek: [-1, 0], kk: [1, 1] }),
      bg: function () { return chair(-48, -14, 28); }, ext: [-50, 40, 0, 32] },
    { id: 'triext', name: 'Extensión de tríceps', grp: 'Pecho y tríceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'v',
      A: P(STAND, { hr: [-10, 6], ek: [0, 1] }), B: P(STAND, { hr: [3, 46], ek: [-1, 0] }) },

    { id: 'goblet', name: 'Sentadilla goblet', grp: 'Piernas', view: 'side', T: 2600, d: [40, 45, 3], db: 'v',
      A: P(STAND, { hr: [14, -10], ek: [0, -1] }),
      B: P(STAND, { hip: [-28, 38], t: 150, hr: [14, -10], ek: [0, -1], kk: [1, 0.3] }) },
    { id: 'lunge', name: 'Zancadas con mancuernas', grp: 'Piernas', view: 'side', T: 2800, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [2, -46], foot2: [0, R], fd2: 90, kk2: [1, 0.2] }),
      B: P(STAND, { hip: [0, 40], hr: [2, -46], foot: [32, R], foot2: [-34, 12], fd2: 40, kk: [1, 0.5], kk2: [0.5, -1] }) },
    { id: 'rdl', name: 'Peso muerto rumano', grp: 'Piernas', view: 'side', T: 3000, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -47] }), B: P(HINGE, { hr: [1, -47] }) },
    { id: 'bridge', name: 'Puente de glúteos', grp: 'Piernas', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(SUP, {}), B: P(SUP, { hip: [-12.6, 36], sh: [-42, 6] }) },
    { id: 'calf', name: 'Elevación de talones', grp: 'Piernas', view: 'side', T: 1600, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -48] }), B: P(STAND, { hip: [0, 76], hr: [4, -48], foot: [0, 14], fd: 40 }) },
    { id: 'sidekick', name: 'Patadas laterales', grp: 'Piernas', view: 'front', T: 2400, d: [40, 45, 3],
      A: P(STAND, { hr: [2, -38], ek: [1, 0], foot: [8, R], foot2: [-8, R], fd: 90, fd2: -90 }),
      B: P(STAND, { hr: [2, -38], ek: [1, 0], foot: [46, 13], foot2: [-8, R], fd: 60, fd2: -90 }) },

    { id: 'situp', name: 'Abdominales en silla', grp: 'Core', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(STAND, { hip: [0, 30], t: 295, hd: -10, hr: [10, -6], foot: [44, R], ek: [0, -1], kk: [0, 1] }),
      B: P(STAND, { hip: [0, 30], t: 205, hd: -10, hr: [10, -6], foot: [44, R], ek: [0, -1], kk: [0, 1] }),
      bg: function () {
        return '<g stroke="currentColor" stroke-opacity=".42" stroke-width="4" stroke-linecap="round" fill="none"><line x1="-8" y1="-25" x2="16" y2="-25"/><line x1="-8" y1="-25" x2="-56" y2="-8"/><line x1="4" y1="-25" x2="4" y2="-1"/><line x1="-40" y1="-15" x2="-40" y2="-1"/></g>';
      }, ext: [-62, 50, 0, 40] },
    { id: 'wheel', name: 'Rueda abdominal', grp: 'Core', view: 'side', T: 3000, d: [40, 45, 3],
      A: P(STAND, { hip: [-30, 38], t: 125, hd: -20, hand: [8, 14], foot: [-62, R], fd: 270, ek: [1, -0.4], kk: [0.5, -1] }),
      B: P(STAND, { hip: [-12.3, 33], t: 100, hd: -20, hand: [66, 12], foot: [-62, R], fd: 270, ek: [1, -0.4], kk: [0.5, -1] }),
      fg: function (J) {
        var w = J.armN[2], c = [w[0], 9.5];
        return '<g><circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="9" fill="none" stroke="currentColor" stroke-width="3.4" ' + OUT.replace('stroke="var(--bgc,#fff)" stroke-width="1.6"', '') + '/><circle cx="' + n1(c[0]) + '" cy="' + n1(-c[1]) + '" r="2.6" fill="currentColor"/></g>';
      }, ext: [-70, 84, 0, 30] },
    { id: 'legraise', name: 'Elevación de piernas', grp: 'Core', view: 'side', T: 2800, d: [40, 45, 3],
      A: P(SUP, { foot: [64, 9], fd: 160 }), B: P(SUP, { foot: [4, 68], fd: 255 }) },
    { id: 'bicycle', name: 'Bicicleta abdominal', grp: 'Core', view: 'side', T: 1800, d: [40, 45, 3],
      A: P(SUP, { hip: [0, R + 2], sh: [-38, 22], hd: -20, hr: [-14, 6], ek: [0, 1], foot: [22, 34], foot2: [64, 16], kk: [-1, 0.6], kk2: [-1, 0.6] }),
      B: P(SUP, { hip: [0, R + 2], sh: [-38, 22], hd: -20, hr: [-14, 6], ek: [0, 1], foot: [64, 16], foot2: [22, 34], kk: [-1, 0.6], kk2: [-1, 0.6] }) },
    { id: 'sideplank', name: 'Plancha lateral', grp: 'Core', view: 'side', T: 2400, d: [40, 45, 3],
      A: P(PLK, sp0, { hand: [52, R], hand2: [31, 76], ek: [-1, -1], ek2: [-1, 0], fd: 40 }),
      B: P(PLK, sp1, { hip: [sp1.hip[0], sp1.hip[1] - 5], hand: [52, R], hand2: [31, 76], ek: [-1, -1], ek2: [-1, 0], fd: 40 }) },
    { id: 'plank', name: 'Plancha frontal', grp: 'Core', view: 'side', T: 3000, d: [40, 45, 3],
      A: P(PLK, fp0, { hand: [49, R] }), B: P(PLK, fp1, { hand: [49, R] }) },

    { id: 'row1', name: 'Remo a una mano', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o', dbN: true,
      A: P(HINGE, { hand: [15, 26], hand2: [44, 34], ek2: [0, 1] }),
      B: P(HINGE, { hand: [-14, 50], hand2: [44, 34], ek2: [0, 1] }),
      bg: function () { return chair(34, 66, 28); }, ext: [-40, 70, 0, 32] },
    { id: 'row2', name: 'Remo inclinado', grp: 'Espalda y bíceps', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(HINGE, { hand: [15, 25] }), B: P(HINGE, { hand: [-2, 54] }) },
    { id: 'superman', name: 'Superman', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], face: -1,
      A: P(SUP, { hd: 0, hand: [-91, R], foot: [66, R], ek: [0, 1], kk: [0, 1] }),
      B: P(SUP, { hip: [0, R], sh: [-41.4, 11.3], hd: -5, hand: [-87, 21], foot: [64, 16], ek: [0, 1], kk: [0, 1] }) },
    { id: 'curl', name: 'Curl de bíceps', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [4, -48], ek: [-1, -0.2] }), B: P(STAND, { hr: [16, -12], ek: [-1, -0.5] }) },
    { id: 'hammer', name: 'Curl martillo', grp: 'Espalda y bíceps', view: 'side', T: 2400, d: [40, 45, 3], db: 'h',
      A: P(STAND, { hr: [4, -48], ek: [-1, -0.2] }), B: P(STAND, { hr: [16, -12], ek: [-1, -0.5] }) },

    { id: 'shpress', name: 'Press de hombros', grp: 'Hombros', view: 'side', T: 2600, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hip: [0, 32], hr: [6, 6], ek: [0, -1], foot: [34, R], kk: [0.5, 1] }),
      B: P(STAND, { hip: [0, 32], hr: [3, 46], ek: [-1, 0], foot: [34, R], kk: [0.5, 1] }),
      bg: function () { return chair(-20, 12, 26, [-20, 72]); }, ext: [-24, 44, 0, 28] },
    { id: 'latraise', name: 'Elevaciones laterales', grp: 'Hombros', view: 'front', T: 2200, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [7, -47], ek: [1, -0.5], foot: [8, R] }), B: P(STAND, { hr: [46, -3], ek: [1, -0.5], foot: [8, R] }) },
    { id: 'frontraise', name: 'Elevaciones frontales', grp: 'Hombros', view: 'side', T: 2200, d: [40, 45, 3], db: 'o',
      A: P(STAND, { hr: [3, -48] }), B: P(STAND, { hr: [47, -2] }) },

    { id: 'rope', name: 'Saltar la cuerda', grp: 'Cardio', view: 'front', T: 700, d: [60, 15, 3],
      A: P(STAND, { hr: [14, -26], ek: [1, -0.5], foot: [8, R] }), B: P(STAND, { hip: [0, 74], hr: [14, -26], ek: [1, -0.5], foot: [8, 12] }),
      fg: function (J, p) {
        var hl = J.armF[2], hr = J.armN[2], ch = 55 + 135 * Math.cos(2 * Math.PI * p);
        return '<path d="M' + pt(hl) + ' Q0,' + n1(-ch) + ' ' + pt(hr) + '" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" opacity=".8"/>';
      }, ext: [-46, 46, -2, 142] },
    { id: 'squatjump', name: 'Sentadillas con salto', grp: 'Cardio', view: 'side', T: 1400, d: [30, 30, 3],
      A: P(STAND, { hip: [-26, 40], t: 145, hr: [-28, -22], ek: [-1, 0], kk: [1, 0.3] }),
      B: P(STAND, { hip: [0, 92], t: 180, hr: [3, 47], foot: [0, 26], fd: 140, ek: [-1, 0], kk: [1, 0.2] }) },
    { id: 'climbers', name: 'Escaladores', grp: 'Cardio', view: 'side', T: 1000, d: [30, 30, 3],
      A: P(CL, { foot: [12, 22], foot2: [-64, 15] }), B: P(CL, { foot: [-64, 15], foot2: [12, 22] }) },

    { id: 'grip', name: 'Apretador de manos', grp: 'Agarre', view: 'side', T: 1400, d: [30, 30, 3],
      A: P(STAND, { hr: [22, -22], ek: [-1, -0.6] }), B: P(STAND, { hr: [22, -22], ek: [-1, -0.6] }),
      fg: function (J, p, s) {
        var w = J.armN[2], g = 8.5 - 5.5 * s, a = [w[0] - 5, w[1] + g], b = [w[0] - 5, w[1] - g], c = [w[0] + 15, w[1]];
        var d = 'M' + pt(a) + ' L' + pt(c) + ' L' + pt(b);
        return '<path d="' + d + '" fill="none" stroke="var(--bgc,#fff)" stroke-width="8" stroke-linecap="round" stroke-linejoin="round"/><path d="' + d + '" fill="none" stroke="currentColor" stroke-width="4.4" stroke-linecap="round" stroke-linejoin="round"/>';
      } }
  ];

  var BYID = {}, GROUPS = [];
  EX.forEach(function (e) { BYID[e.id] = e; if (GROUPS.indexOf(e.grp) < 0) GROUPS.push(e.grp); });

  function pose(ex, p) {
    if (ex.fn) return ex.fn(p);
    var s = 0.5 - 0.5 * Math.cos(2 * Math.PI * p);
    return mix(ex.A, ex.B, s);
  }
  function sOf(p) { return 0.5 - 0.5 * Math.cos(2 * Math.PI * p); }

  var vbCache = {};
  function bbox(ex, sex) {
    var key = ex.id + sex; if (vbCache[key]) return vbCache[key];
    var x0 = 1e9, x1 = -1e9, h0 = 0, h1 = 0, i, B = BODY[sex];
    function acc(c, r) { r = r || 6; x0 = Math.min(x0, c[0] - r); x1 = Math.max(x1, c[0] + r); h0 = Math.min(h0, c[1] - r); h1 = Math.max(h1, c[1] + r); }
    for (i = 0; i <= 24; i++) {
      var J = solve(pose(ex, i / 24), ex.view, sex);
      [J.hip, J.sh, J.armN[1], J.armN[2], J.armF[1], J.armF[2], J.legN[1], J.legN[2], J.legN[3], J.legF[1], J.legF[2], J.legF[3]].forEach(function (c) { acc(c, 7); });
      acc(J.head, B.head + 12);
      if (J.view === 'front') { acc([J.sh[0] + B.sw, J.sh[1]], 6); acc([J.sh[0] - B.sw, J.sh[1]], 6); }
    }
    if (ex.ext) { x0 = Math.min(x0, ex.ext[0]); x1 = Math.max(x1, ex.ext[1]); h0 = Math.min(h0, ex.ext[2]); h1 = Math.max(h1, ex.ext[3]); }
    h0 = Math.min(h0, -6);
    var pad = 14, w = x1 - x0 + pad * 2, h = h1 - h0 + pad * 2, cx = (x0 + x1) / 2, AR = 1.8;
    if (w / h < AR) w = h * AR; else h = w / AR;
    var bottom = h0 - pad;          // el piso queda abajo
    var vb = { x: cx - w / 2, y: -(bottom + h), w: w, h: h, x0: x0, x1: x1 };
    return (vbCache[key] = vb);
  }

  function viewBox(id, sex) { var v = bbox(BYID[id], sex); return n1(v.x) + ' ' + n1(v.y) + ' ' + n1(v.w) + ' ' + n1(v.h); }
  function render(id, sex, p, still) {
    var ex = BYID[id], v = bbox(ex, sex), s = sOf(p);
    var J = solve(pose(ex, p), ex.view, sex);
    var ground = '<line x1="' + n1(v.x - 400) + '" y1="1.5" x2="' + n1(v.x + v.w + 400) + '" y2="1.5" stroke="currentColor" stroke-opacity=".38" stroke-width="3"/>';
    return ground + drawFigure(J, ex, sex, p, s);
  }
  function thumb(id, sex) {
    var ex = BYID[id], p = ex.tp === undefined ? 0.5 : ex.tp;
    return '<svg viewBox="' + viewBox(id, sex) + '" preserveAspectRatio="xMidYMid meet" aria-hidden="true" focusable="false">' + render(id, sex, p) + '</svg>';
  }
  function check() { // diagnóstico: alcance de brazos/piernas
    var rep = [];
    EX.forEach(function (ex) {
      ['m', 'f'].forEach(function (sex) {
        var worst = 0;
        for (var i = 0; i <= 24; i++) worst = Math.max(worst, solve(pose(ex, i / 24), ex.view, sex).short);
        if (worst > 0.8) rep.push(ex.id + '/' + sex + ': ' + worst.toFixed(1));
      });
    });
    return rep;
  }

  root.LochFig = { list: EX, byId: BYID, groups: GROUPS, viewBox: viewBox, render: render, thumb: thumb, check: check, T: function (id) { return BYID[id].T; } };
})(window);
