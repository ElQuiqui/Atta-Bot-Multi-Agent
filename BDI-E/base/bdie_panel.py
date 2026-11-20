"""
bdie_panel.py — paneles BDI-E de la GUI: el mapa y la mente del enjambre

  MapaBdieWidget   la cuadrícula tal como la ve un robot (o todos): feromona
                   repelente, cobertura acumulada o zonas, con los robots, sus
                   metas, el objetivo y la base encima.
  MenteBdieWidget  una fila por robot con lo que está pensando (fase, deseo,
                   plan, meta, zona, objetivo, último evento) y las alertas de
                   las verificaciones en vivo (bdie_checks.py).

Leen del BdieMonitor de la base (base.bdie) con snapshot(): nunca tocan
directamente los datos del hilo de red. Todo lo que se ve aquí queda también en
BdieLogs/.
"""
import math
import time

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
                             QCheckBox, QTableWidget, QTableWidgetItem, QHeaderView,
                             QAbstractItemView, QListWidget, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF

# Los mismos colores por robot que el mapa de cobertura de la base (BGR → RGB).
_COLORES_BGR = [(220, 70, 30), (30, 180, 30), (30, 30, 210), (190, 40, 190),
                (20, 190, 190), (20, 130, 220)]


def color_robot(rid, alfa=255):
    b, g, r = _COLORES_BGR[int(rid) % len(_COLORES_BGR)]
    return QColor(r, g, b, alfa)


CAPAS = ["Repelente", "Cobertura", "Zonas"]
FONDO = QColor('#101216')
REJILLA = QColor('#34373d')
TEXTO = QColor('#e6e6e6')
TENUE = QColor('#9aa0a8')


def _rampa(v, capa):
    """0..255 → color. Repelente en ámbar."""
    t = max(0.0, min(1.0, v / 255.0))
    return QColor(int(16 + 230 * t), int(18 + 140 * t), int(22 + 10 * t))


class _Lienzo(QWidget):
    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.setMinimumSize(360, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), FONDO)
        try:
            self.panel.pintar(p, self.width(), self.height())
        finally:
            p.end()


class MapaBdieWidget(QWidget):
    def __init__(self, base):
        super().__init__()
        self.base = base
        self.snap = None
        caja = QVBoxLayout(self)
        caja.setContentsMargins(4, 4, 4, 4)
        fila = QHBoxLayout()
        fila.addWidget(QLabel('Vista de'))
        self.cbRobot = QComboBox()
        self.cbRobot.addItem('todos (máx.)', userData=None)
        fila.addWidget(self.cbRobot)
        fila.addWidget(QLabel('capa'))
        self.cbCapa = QComboBox()
        self.cbCapa.addItems(CAPAS)
        fila.addWidget(self.cbCapa)
        self.chkMetas = QCheckBox('metas')
        self.chkMetas.setChecked(True)
        fila.addWidget(self.chkMetas)
        self.chkZonas = QCheckBox('bordes de zona')
        self.chkZonas.setChecked(True)
        fila.addWidget(self.chkZonas)
        fila.addStretch()
        caja.addLayout(fila)
        self.lienzo = _Lienzo(self)
        caja.addWidget(self.lienzo, stretch=1)
        self.pie = QLabel('Sin telemetría BDI-E todavía (BDIE_TELEM|2 en los robots).')
        self.pie.setObjectName('tenue')
        caja.addWidget(self.pie)
        for w in (self.cbRobot, self.cbCapa):
            w.currentIndexChanged.connect(self.lienzo.update)
        for w in (self.chkMetas, self.chkZonas):
            w.toggled.connect(self.lienzo.update)
        self._t = QTimer(self)
        self._t.timeout.connect(self.refrescar)
        self._t.start(500)

    def refrescar(self):
        mon = getattr(self.base, 'bdie', None)
        if mon is None:
            return
        self.snap = mon.snapshot()
        ids = sorted(self.snap['mapas'])
        actuales = [self.cbRobot.itemData(i) for i in range(1, self.cbRobot.count())]
        if ids != actuales:
            sel = self.cbRobot.currentData()
            self.cbRobot.blockSignals(True)
            while self.cbRobot.count() > 1:
                self.cbRobot.removeItem(1)
            for i in ids:
                self.cbRobot.addItem(f'R{i}', userData=i)
            j = self.cbRobot.findData(sel)
            self.cbRobot.setCurrentIndex(j if j >= 0 else 0)
            self.cbRobot.blockSignals(False)
        self.lienzo.update()

    # ── dibujo ────────────────────────────────────────────────────────────────
    def _frescos(self):
        """Los mapas de robots que siguen hablando: un robot caído deja su
        último mapa congelado, y compararlo con los vivos inventa desacuerdos."""
        mapas = self.snap['mapas']
        if not mapas:
            return {}
        tmax = max(m['tlog'] for m in mapas.values())
        return {i: m for i, m in mapas.items() if tmax - m['tlog'] <= 3.0}

    def _mapaVista(self):
        """El mapa que se dibuja: el de un robot, o el máximo celda a celda."""
        rid = self.cbRobot.currentData()
        if rid is not None and rid in self.snap['mapas']:
            return self.snap['mapas'][rid], rid
        mapas = self._frescos()
        if not mapas:
            return None, None
        ref = mapas[min(mapas)]
        m = dict(ref)
        for campo in ("R", "V"):
            m[campo] = [max(x[campo][k] for x in mapas.values() if len(x[campo]) > k)
                        for k in range(len(ref[campo]))]
        return m, None

    def pintar(self, p, W, H):
        if not self.snap or not self.snap['mapas']:
            p.setPen(TENUE)
            p.drawText(QRectF(0, 0, W, H), Qt.AlignCenter,
                       'Esperando BMAP de los robots…')
            return
        m, rid = self._mapaVista()
        filas, cols = m['filas'], m['cols']
        lado = min((W - 20) / cols, (H - 20) / filas)
        x0 = (W - lado * cols) / 2
        y0 = (H - lado * filas) / 2
        capa = self.cbCapa.currentText()

        def celdaRect(k):
            r, c = divmod(k, cols)
            return QRectF(x0 + c * lado, y0 + r * lado, lado, lado)

        def mm2px(x, y):
            return QPointF(x0 + (x - m['ox']) / m['celda'] * lado,
                           y0 + (y - m['oy']) / m['celda'] * lado)

        def centro(k):
            return celdaRect(k).center()

        # 1 · la capa
        zonaDe = {}
        for i, (r0, r1, c0, c1) in enumerate(m['zonas']):
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    zonaDe[r * cols + c] = i
        duenoZona = {z: r for r, z in m['de'].items()}
        for k in range(filas * cols):
            if capa == "Repelente":
                col = _rampa(m['R'][k], capa)
            elif capa == "Cobertura":
                col = QColor('#4a5563') if m['V'][k] else QColor('#181a1f')
            else:
                z = zonaDe.get(k)
                col = color_robot(duenoZona[z], 90) if z in duenoZona else QColor('#181a1f')
            p.fillRect(celdaRect(k), col)
            if m['B'][k]:
                p.setPen(QPen(QColor('#ff5f57'), 2))
                rc = celdaRect(k).adjusted(3, 3, -3, -3)
                p.drawLine(rc.topLeft(), rc.bottomRight())
                p.drawLine(rc.topRight(), rc.bottomLeft())
        p.setPen(QPen(REJILLA, 1))
        for c in range(cols + 1):
            p.drawLine(QPointF(x0 + c * lado, y0), QPointF(x0 + c * lado, y0 + filas * lado))
        for r in range(filas + 1):
            p.drawLine(QPointF(x0, y0 + r * lado), QPointF(x0 + cols * lado, y0 + r * lado))

        # 1b · DESACUERDO de zonas entre réplicas (V1): en la vista «todos», las
        # celdas cuyo dueño no es el mismo para todos los robots se rayan en rojo.
        # Es la propiedad central de P3 hecha visible.
        desacuerdo = 0
        if rid is None and capa == "Zonas":
            vistas = []
            for x in self._frescos().values():
                zd = {}
                dz = {z: r for r, z in x['de'].items()}
                for i, (r0, r1, c0, c1) in enumerate(x['zonas']):
                    for r in range(r0, r1 + 1):
                        for c in range(c0, c1 + 1):
                            zd[r * cols + c] = dz.get(i)
                vistas.append(zd)
            p.setPen(QPen(QColor('#ff3b30'), 2))
            for k in range(filas * cols):
                if len({v.get(k) for v in vistas}) > 1:
                    desacuerdo += 1
                    rc = celdaRect(k)
                    p.drawLine(rc.bottomLeft(), rc.topRight())
        self._desacuerdo = desacuerdo

        # 2 · bordes de zona, con el color del robot al que esta vista se la asigna
        if self.chkZonas.isChecked():
            for i, (r0, r1, c0, c1) in enumerate(m['zonas']):
                dueno = duenoZona.get(i)
                col = color_robot(dueno) if dueno is not None else TENUE
                p.setPen(QPen(col, 3))
                p.setBrush(Qt.NoBrush)
                p.drawRect(QRectF(x0 + c0 * lado + 1.5, y0 + r0 * lado + 1.5,
                                  (c1 - c0 + 1) * lado - 3, (r1 - r0 + 1) * lado - 3))
                p.setPen(col)
                p.setFont(QFont('', 8, QFont.Bold))
                p.drawText(QPointF(x0 + c0 * lado + 5, y0 + r0 * lado + 13),
                           f'Z{i}' + (f'→R{dueno}' if dueno is not None else ''))

        think = self.snap['think']
        # 3 · la base y el objetivo
        if self.snap['home']:
            q = mm2px(*self.snap['home'])
            p.setPen(QPen(TEXTO, 2))
            p.setBrush(QColor('#24262b'))
            p.drawRect(QRectF(q.x() - 8, q.y() - 8, 16, 16))
            p.drawText(QPointF(q.x() - 4, q.y() + 4), 'B')
        cobj = next((t['cobj'] for t in think.values() if t.get('obj') == 1 and t.get('cobj', -1) >= 0), None)
        if cobj is not None and cobj < filas * cols:
            q = centro(cobj)
            def punta(i):
                r = lado * 0.42 * (0.5 if i % 2 else 1.0)
                a = math.pi / 5 * i - math.pi / 2
                return QPointF(q.x() + r * math.cos(a), q.y() + r * math.sin(a))
            est = QPolygonF([punta(i) for i in range(10)])
            p.setPen(QPen(QColor('#ffffff'), 1))
            p.setBrush(QColor('#ff3b30'))
            p.drawPolygon(est)
        if self.snap.get('objetivo_real'):
            q = mm2px(*self.snap['objetivo_real'])
            p.setPen(QPen(QColor('#ff3b30'), 2, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(q, 10, 10)

        # 4 · los robots: pose de cámara si hay, si no la que cree el agente
        for i, t in sorted(think.items()):
            cam = self.snap['camara'].get(i)
            x, y = (cam[0], cam[1]) if cam else (t.get('x'), t.get('y'))
            if not isinstance(x, (int, float)):
                continue
            q = mm2px(x, y)
            callado = (max(tt['tlog'] for tt in think.values()) - t['tlog']) > 3.0
            col = QColor('#5a5f68') if callado else color_robot(i)
            meta = t.get('meta', -1)
            if (self.chkMetas.isChecked() and not callado and isinstance(meta, int)
                    and 0 <= meta < filas * cols):
                p.setPen(QPen(col, 2, Qt.DashLine))
                p.drawLine(q, centro(meta))
                g = centro(meta)
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(g, lado * 0.22, lado * 0.22)
            p.setPen(QPen(QColor('#ffffff') if i == rid else QColor('#000000'), 2))
            p.setBrush(col)
            radio = max(6.0, lado * 0.33)
            p.drawEllipse(q, radio, radio)
            p.setPen(QColor('#ffffff'))
            p.setFont(QFont('', 8, QFont.Bold))
            p.drawText(QRectF(q.x() - radio, q.y() - radio, 2 * radio, 2 * radio),
                       Qt.AlignCenter, str(i))

        # pie
        vista = f'R{rid}' if rid is not None else 'todos (máximo celda a celda)'
        cob = sum(m['V'])
        if getattr(self, '_desacuerdo', 0):
            vista += f' · ⚠ {self._desacuerdo} celdas con dueño distinto según quién mire (V1)'
        self.pie.setText(f'Vista: {vista} · capa {capa} · {filas}×{cols} celdas de '
                         f'{m["celda"]:.0f} mm · cobertura {cob}/{filas * cols} · '
                         f'{len(m["zonas"])} zonas {m["de"]}')


class MenteBdieWidget(QWidget):
    COLS = ['Robot', 'Fase', 'Deseo', 'Plan', 'Meta', 'Celda', 'Zona', 'Objetivo',
            'Vivos', 'Cob.', 'Delib.', 'Último evento', 'Edad']

    def __init__(self, base):
        super().__init__()
        self.base = base
        caja = QVBoxLayout(self)
        caja.setContentsMargins(4, 4, 4, 4)
        self.tabla = QTableWidget(0, len(self.COLS))
        self.tabla.setHorizontalHeaderLabels(self.COLS)
        self.tabla.verticalHeader().setVisible(False)
        self.tabla.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tabla.setSelectionMode(QAbstractItemView.NoSelection)
        self.tabla.setFont(QFont('Monospace', 9))
        cab = self.tabla.horizontalHeader()
        cab.setSectionResizeMode(QHeaderView.ResizeToContents)
        cab.setStretchLastSection(True)
        self.tabla.setMinimumHeight(165)
        caja.addWidget(self.tabla, stretch=3)
        self.alertas = QListWidget()
        self.alertas.setFont(QFont('Monospace', 9))
        self.alertas.setMaximumHeight(80)
        caja.addWidget(QLabel('Alertas de las verificaciones (V1–V10, ver PRUEBAS_observacion.md)'))
        caja.addWidget(self.alertas, stretch=1)
        self._t = QTimer(self)
        self._t.timeout.connect(self.refrescar)
        self._t.start(500)
        self._nAlertas = 0

    def refrescar(self):
        mon = getattr(self.base, 'bdie', None)
        if mon is None:
            return
        ahora = self.base._tlog() if hasattr(self.base, '_tlog') else mon.ahora()
        mon.tick(ahora)
        s = mon.snapshot()
        filas = sorted(s['think'].items())
        self.tabla.setRowCount(len(filas))
        for f, (rid, t) in enumerate(filas):
            edad = ahora - t['tlog']
            vivos = t.get('vivos', '0')
            try:
                n = bin(int(str(vivos), 16)).count('1')
            except ValueError:
                n = '?'
            obj = '—'
            if t.get('obj') == 1:
                obj = f"R{t.get('dueno')} @{t.get('cobj')}" + (' ★' if t.get('dueno') == rid else '')
            meta = t.get('meta', -1)
            valores = [f'R{rid}', t.get('fase', ''), t.get('deseo', ''), t.get('plan', ''),
                       '—' if meta in (-1, None) else str(meta), str(t.get('celda', '')),
                       str(t.get('zona', '')) + ('' if t.get('enzona') else ' (fuera)'),
                       obj, str(n), str(t.get('cob', '')), str(t.get('delib', '')),
                       f"{t.get('ev', '')} ({max(0, (t.get('t', 0) - t.get('tev', 0))) / 1000:.0f} s)",
                       f'{edad:.1f} s']
            for c, v in enumerate(valores):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignCenter)
                if c == 0:
                    it.setForeground(color_robot(rid))
                if edad > 3.0:
                    it.setForeground(QColor('#ff5f57' if c == 12 else '#9aa0a8'))
                elif c == 1 and v == 'SEGURO':
                    it.setForeground(QColor('#ff5f57'))
                self.tabla.setItem(f, c, it)
        if len(s['alertas']) != self._nAlertas:
            self._nAlertas = len(s['alertas'])
            self.alertas.clear()
            for (tl, r, cod, txt) in reversed(s['alertas'][-50:]):
                self.alertas.addItem(f'{tl:7.1f}s  {cod}  {txt}')
