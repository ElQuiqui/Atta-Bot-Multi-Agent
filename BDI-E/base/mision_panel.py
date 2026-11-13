"""Barra de misión de la GUI (12-oct-2026).

Lo que antes había que deducir leyendo el log, a la vista y en una sola franja:

  - qué color busca el enjambre, y qué robots CONFIRMARON ese color
    (cada robot contesta 'TARGET_COLOR: <color>' al recibirlo);
  - si hay una sesión en curso, cuánto lleva contra el tope de 20 min;
  - la fase de cada robot, su zona y si ya conoce el objetivo;
  - la cobertura acumulada y quién encontró el objetivo, dónde.

El botón «Iniciar misión» manda el color ANTES del SESSION|START, en la misma
orden (BASE.SESSION|START|x|y|color). El color no se guarda en el robot: si uno
se reinicia vuelve a rojo, así que mandarlo en cada inicio es lo único seguro.
"""
import re
import time

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QComboBox,
    QSpinBox, QLineEdit, QFrame, QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPixmap, QIcon

from bdie_panel import color_robot

COLORES = {            # nombre en el protocolo → color de pantalla
    'rojo':  '#ff4d4d',
    'verde': '#3ecf8e',
    'azul':  '#4a9eff',
}

# Color de cada fase, para leerla de un vistazo.
FASES = {
    'IDLE':      ('#9aa0a8', 'en espera'),
    'DESPLEGAR': ('#c792ea', 'yendo a su zona'),
    'BUSCAR':    ('#4a9eff', 'buscando'),
    'CONGREGAR': ('#f5a623', 'congregando'),
    'VOLVER':    ('#3ecf8e', 'volviendo'),
    'FIN':       ('#3ecf8e', 'terminó'),
    'SEGURO':    ('#ff5f57', 'detenido'),
}

TOPE_MISION_S = 20 * 60          # restricción dura del cliente

_RE_ACK = re.compile(r'Mensaje de (\S+): TARGET_COLOR: (\w+)')


def _icono(hexcolor):
    pm = QPixmap(14, 14)
    pm.fill(QColor(hexcolor))
    return QIcon(pm)


class _Chip(QLabel):
    """Etiqueta con fondo redondeado, para estados cortos."""
    def __init__(self, texto='', color='#34373d', fg='#e6e6e6'):
        super().__init__(texto)
        self.setAlignment(Qt.AlignCenter)
        self.pintar(texto, color, fg)

    def pintar(self, texto, color, fg='#0d1117'):
        self.setText(texto)
        self.setStyleSheet(f'background:{color}; color:{fg}; border-radius:10px;'
                           f'padding:3px 10px; font-weight:600;')


class MisionBar(QFrame):
    def __init__(self, base, dispatch, log):
        super().__init__()
        self.base = base
        self._dispatch = dispatch
        self._log = log
        self.setObjectName('panel')
        self._tInicio = None         # time.time() del START
        self._estado = 'SIN SESIÓN'
        self._colorSesion = None     # color con que arrancó la sesión en curso
        self._acks = {}              # nombre del robot → color confirmado

        raiz = QVBoxLayout(self)
        raiz.setContentsMargins(10, 6, 10, 6)
        raiz.setSpacing(4)

        # ── fila 1: estado + controles de inicio ─────────────────────────────
        f1 = QHBoxLayout()
        f1.setSpacing(8)
        titulo = QLabel('Misión')
        titulo.setFont(QFont('', 11, QFont.Bold))
        f1.addWidget(titulo)
        self._chipEstado = _Chip('SIN SESIÓN', '#34373d', '#e6e6e6')
        self._chipEstado.setMinimumWidth(120)   # 24-oct: que la fila no se mueva al cambiar el texto
        f1.addWidget(self._chipEstado)
        self._reloj = QLabel('—')
        self._reloj.setFont(QFont('Monospace', 12, QFont.Bold))
        self._reloj.setMinimumWidth(175)
        f1.addWidget(self._reloj)
        self._chipColor = _Chip('busca: —', '#34373d', '#e6e6e6')
        self._chipColor.setToolTip('Color objetivo de la sesión en curso y cuántos '
                                   'robots lo confirmaron')
        self._chipColor.setMinimumWidth(170)
        f1.addWidget(self._chipColor)
        f1.addStretch()

        f1.addWidget(QLabel('Color'))
        self._color = QComboBox()
        for nombre, hx in COLORES.items():
            self._color.addItem(_icono(hx), nombre)
        self._color.setToolTip('Color que van a buscar. Se manda a todos los robots '
                               'al iniciar la misión.')
        f1.addWidget(self._color)
        f1.addWidget(QLabel('Base (mm)'))
        self._bx = QSpinBox(); self._bx.setRange(0, 4000); self._bx.setValue(300)
        self._by = QSpinBox(); self._by.setRange(0, 4000); self._by.setValue(300)
        for sb in (self._bx, self._by):
            sb.setSingleStep(50)
            sb.setToolTip('Punto de retorno, en el marco del ArUco de referencia')
            f1.addWidget(sb)
        f1.addWidget(QLabel('Objeto'))
        self._obj = QLineEdit()
        self._obj.setPlaceholderText('x y (la cámara)')
        self._obj.setMaximumWidth(110)
        self._obj.setToolTip('Opcional desde el 24-oct: la cámara detecta el cubo sola y lo '
                             'registra. Escribirlo acá solo sirve si la cámara no lo ve. Queda en '
                             'el log (objetivo_real); los robots NO lo reciben.')
        f1.addWidget(self._obj)
        self._btnIniciar = QPushButton('▶  Iniciar misión')
        self._btnIniciar.setObjectName('primario')
        # 24-oct: con «clicked» hacía falta un segundo clic después de escribir
        # en «Objeto»: el primer clic saca el foco del campo y el refresco de
        # 500 ms reacomoda la fila entre el press y el release, así que el
        # release cae fuera y Qt no emite clicked. «pressed» dispara al bajar el
        # botón; el antirrebote evita dos START seguidos.
        self._tUltimoInicio = 0.0
        self._btnIniciar.pressed.connect(self._iniciarPresionado)
        f1.addWidget(self._btnIniciar)
        self._btnAbortar = QPushButton('■  Abortar')
        self._btnAbortar.setObjectName('peligro')
        self._btnAbortar.clicked.connect(self.abortar)
        f1.addWidget(self._btnAbortar)
        raiz.addLayout(f1)

        # ── fila 2: un chip por robot + cobertura + hallazgo ─────────────────
        f2 = QHBoxLayout()
        f2.setSpacing(8)
        self._filaRobots = QHBoxLayout()
        self._filaRobots.setSpacing(6)
        f2.addLayout(self._filaRobots)
        f2.addStretch()
        self._cob = QLabel('Cobertura —')
        self._cob.setFont(QFont('', 10, QFont.Bold))
        f2.addWidget(self._cob)
        self._hallazgo = QLabel('')
        self._hallazgo.setFont(QFont('', 10, QFont.Bold))
        f2.addWidget(self._hallazgo)
        raiz.addLayout(f2)
        self._chipsRobot = {}

        self._t = QTimer(self)
        self._t.timeout.connect(self.refrescar)
        self._t.start(500)

    # ── acciones ─────────────────────────────────────────────────────────────
    def _iniciarPresionado(self):
        if time.time() - self._tUltimoInicio < 1.0:
            return
        self._tUltimoInicio = time.time()
        self.iniciar()

    def iniciar(self):
        color = self._color.currentText()
        x, y = self._bx.value(), self._by.value()
        if self._estado in ('EN CURSO', 'HALLADO'):
            r = QMessageBox.question(self, 'Misión en curso',
                                     'Ya hay una misión en curso. ¿Reiniciarla?')
            if r != QMessageBox.Yes:
                return
        self._acks = {}
        self._colorSesion = color
        self._dispatch('BASE', f'SESSION|START|{x}|{y}|{color}')
        obj = self._obj.text().replace(',', ' ').split()
        if len(obj) == 2:
            try:
                ox, oy = float(obj[0]), float(obj[1])
                mon = getattr(self.base, 'bdie', None)
                if mon is not None:
                    mon.set_objetivo_real(ox, oy)
                    self._log(f'[Misión] objetivo real registrado en el log: ({ox:.0f}, {oy:.0f})')
            except ValueError:
                self._log('[Misión] «Objeto» debe ser dos números en mm; no se registró.')
        self._tInicio = time.time()
        self._estado = 'EN CURSO'

    def abortar(self):
        self._dispatch('BASE', 'SESSION|ABORT')
        if self._estado in ('EN CURSO', 'HALLADO'):
            self._estado = 'ABORTADA'

    def onLog(self, msg):
        """La GUI le pasa cada línea del log: de acá salen los acuses de color."""
        m = _RE_ACK.search(msg)
        if m:
            self._acks[m.group(1)] = m.group(2)

    # ── refresco ─────────────────────────────────────────────────────────────
    def refrescar(self):
        # reloj y estado
        if self._estado in ('EN CURSO', 'HALLADO') and self._tInicio:
            dt = time.time() - self._tInicio
            mm, ss = divmod(int(dt), 60)
            self._reloj.setText(f'{mm:02d}:{ss:02d} / 20:00')
            self._reloj.setStyleSheet('color:#ff5f57;' if dt > TOPE_MISION_S * 0.85 else '')
        elif self._estado == 'SIN SESIÓN':
            self._reloj.setText('—')
        colEst = {'COMPLETADA': ('#3ecf8e', '#0d1117'),
                  'SIN SESIÓN': ('#34373d', '#e6e6e6'), 'EN CURSO': ('#4a9eff', '#0d1117'),
                  'ABORTADA': ('#ff5f57', '#0d1117'), 'HALLADO': ('#3ecf8e', '#0d1117')}
        bg, fg = colEst.get(self._estado, ('#34373d', '#e6e6e6'))
        self._chipEstado.pintar(self._estado, bg, fg)

        # color buscado + confirmaciones
        robots = getattr(self.base, 'robots', {}) or {}
        nombres = {getattr(r, 'name', str(rid)): rid for rid, r in robots.items()}
        c = self._colorSesion
        if c:
            # Solo cuentan los robots registrados; un robot encendido y sin
            # registrar también contesta (17-oct: salía «4/3 confirmaron»).
            ok = sum(1 for n, v in self._acks.items() if v == c and n in nombres)
            mal = [n for n, v in self._acks.items() if v != c and n in nombres]
            extra = [n for n in self._acks if n not in nombres]
            txt = f'busca: {c.upper()}  ({ok}/{len(robots) or "?"} confirmaron)'
            if mal:
                txt += '  ⚠ ' + ', '.join(mal)
            if extra:
                txt += f'  ⚠ {len(extra)} sin registrar'
            self._chipColor.pintar(txt, COLORES.get(c, '#34373d'), '#0d1117')
        else:
            self._chipColor.pintar(f'busca: {self._color.currentText().upper()} (sin enviar)',
                                   '#34373d', COLORES.get(self._color.currentText(), '#e6e6e6'))

        # chips por robot, desde los THINK del monitor BDI-E
        mon = getattr(self.base, 'bdie', None)
        think = {}
        if mon is not None:
            try:
                think = mon.snapshot().get('think', {})
            except Exception:
                think = {}
        ahora = self.base._tlog() if hasattr(self.base, '_tlog') else 0.0
        cobMax, hallado = 0, None
        for rid, t in sorted(think.items()):
            chip = self._chipsRobot.get(rid)
            if chip is None:
                chip = QLabel()
                chip.setTextFormat(Qt.RichText)
                chip.setFont(QFont('', 9, QFont.Bold))
                self._filaRobots.addWidget(chip)
                self._chipsRobot[rid] = chip
            fase = str(t.get('fase', ''))
            col, desc = FASES.get(fase, ('#9aa0a8', fase.lower()))
            edad = ahora - t.get('tlog', ahora)
            nombre = next((n for n, i in nombres.items() if str(i) == str(rid)), f'R{rid}')
            ack = self._acks.get(nombre)
            marca = ''
            if c:
                marca = ' ✓' if ack == c else (' ✗' if ack else ' ?')
            zona = t.get('zona', -1)
            punto = color_robot(int(rid)).name() if str(rid).isdigit() else col
            texto = (f'<span style="color:{punto};">●</span> <b>R{rid}</b> {desc}'
                     + (f' · Z{zona}' if zona not in (-1, None, '') else '') + marca)
            if edad > 3.0:
                texto += f' · sin datos {edad:.0f} s'
                col = '#6b7078'
            chip.setText(texto)
            chip.setStyleSheet(f'color:{col}; border:1px solid {col}; border-radius:9px;'
                               f'padding:2px 8px;')
            chip.setToolTip(f"plan {t.get('plan', '')} · meta {t.get('meta', '')} · "
                            f"celda {t.get('celda', '')} · último evento {t.get('ev', '')}")
            try:
                cobMax = max(cobMax, int(t.get('cob', 0)))
            except (TypeError, ValueError):
                pass
            if t.get('obj') == 1 and hallado is None:
                hallado = (t.get('dueno'), t.get('cobj'))
        n = 160
        try:
            m = next(iter(mon.snapshot().get('mapas', {}).values()))
            n = int(m.get('filas', 10)) * int(m.get('cols', 16))
        except Exception:
            pass
        self._cob.setText(f'Cobertura {cobMax}/{n} ({100 * cobMax / n:.0f} %)' if think else 'Cobertura —')
        # 17-oct: todos los robots en FIN = volvieron a la base, misión cumplida
        if think and self._estado in ('EN CURSO', 'HALLADO') and \
                all(str(t.get('fase', '')) == 'FIN' for t in think.values()):
            self._estado = 'COMPLETADA'
        if hallado:
            self._hallazgo.setText(f'★ Objetivo hallado por R{hallado[0]} en la celda {hallado[1]}')
            self._hallazgo.setStyleSheet('color:#3ecf8e;')
            if self._estado == 'EN CURSO':
                self._estado = 'HALLADO'
        else:
            self._hallazgo.setText('')
