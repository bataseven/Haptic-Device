# PyInstaller build of the desktop app as a single .exe that runs without Python.
# Build it in this folder with:  pyinstaller HapticDeviceInterface.spec
# The .exe is written to dist/HapticDeviceInterface.exe

a = Analysis(
    ['app.pyw'],
    # The windows are loaded from .ui files and images next to the UI modules, so they are bundled at the same paths
    datas=[
        ('UI/*.ui', 'UI'),
        ('UI/images/*.png', 'UI/images'),
    ],
    # matplotlib, matplotx and serial are only used by the standalone UI/serial_animator.py. The others are optional
    # imports of pyqtgraph that the app doesn't need, and would make the .exe much larger if they are installed.
    excludes=['matplotlib', 'matplotx', 'serial', 'tkinter', 'scipy', 'pandas', 'IPython', 'jupyter',
              'PyQt6', 'PySide2', 'PySide6', 'OpenGL', 'cupy', 'numba', 'h5py'],
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    name='HapticDeviceInterface',
    icon='UI/images/assembled.ico',
    console=False,
    upx=False,
)
