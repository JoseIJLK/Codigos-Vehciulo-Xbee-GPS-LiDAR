import tkinter as tk
from tkinter import ttk, messagebox
from tkintermapview import TkinterMapView
import serial, json, threading, queue, time, csv, os, math
from datetime import datetime

UMBRAL_ALERTA = 40  

ESCALA_LIDAR = 2.5
ESCALA_MIN = 0.8
ESCALA_MAX = 6.0
PASO_ZOOM = 0.3

INTERVALO_REGISTRO = 1.0
ultimo_registro = 0

MAX_PUNTOS_TRAYECTORIA = 500
MAX_NUBE = 3000

modo_auto = False
modo_lidar_nube = False    
ultima_direccion = "S"

DIRECCIONES_MANUAL = {
    "F": "adelante",
    "B": "atras",
    "L": "izquierda",
    "R": "derecha",
    "S": "detenido",
    "M": "detenido"
}

direccion_auto = "detenido"

xbee_serial = None
serial_queue = queue.Queue()
stop_serial_thread = threading.Event()

trayectoria = []
ultimo_gps = None
obstaculo_mas_cercano = None
nube_lidar = []            

tomando_datos = False
registro_temporal = []

marker_actual = None
path_actual = None

tiempo_inicio_registro = None

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = None

def abrir_conexion_serial(puerto, baud=9600):
    global xbee_serial
    try:
        cerrar_conexion_serial()
        xbee_serial = serial.Serial(puerto, baud, timeout=1)
        status_label.config(text=f"Conectado a {puerto}", fg="green")
        stop_serial_thread.clear()
        threading.Thread(target=leer_serial, daemon=True).start()
        time.sleep(0.3)
        enviar_comando("M")
    except Exception as e:
        status_label.config(text=f"Error: {e}", fg="red")

def cerrar_conexion_serial():
    global xbee_serial
    stop_serial_thread.set()
    if xbee_serial and xbee_serial.is_open:
        xbee_serial.close()
    status_label.config(text="Desconectado", fg="red")

def leer_serial():
    while not stop_serial_thread.is_set() and xbee_serial and xbee_serial.is_open:
        try:
            line = xbee_serial.readline().decode(errors="ignore").strip()
            if line:
                serial_queue.put(line)
        except:
            pass

def procesar_linea(line):
    global ultimo_gps, obstaculo_mas_cercano, direccion_auto

    try:
        data = json.loads(line)
    except:
        return

    if "gps" in data:
        gps = data["gps"]
        if gps.get("lat") and gps.get("lon"):
            ultimo_gps = gps
            trayectoria.append((gps["lat"], gps["lon"]))
            if len(trayectoria) > MAX_PUNTOS_TRAYECTORIA:
                trayectoria.pop(0)

    if "auto_dir" in data:
        direccion_auto = data["auto_dir"]

    if "lidar" in data:
        min_dist = float("inf")
        obstaculo_mas_cercano = None

        for p in data["lidar"]:
            try:
                x = float(p["x"])
                y = float(p["y"])
                dist = math.hypot(x, y) / 10.0

                nube_lidar.append((x, y))
                if len(nube_lidar) > MAX_NUBE:
                    nube_lidar.pop(0)

                if dist < min_dist:
                    min_dist = dist
                    obstaculo_mas_cercano = (x, y, dist)
            except:
                pass


def enviar_comando(cmd):
    global ultima_direccion
    ultima_direccion = cmd
    if xbee_serial and xbee_serial.is_open:
        try:
            xbee_serial.write(cmd.encode())
        except:
            pass

def cambiar_modo():
    global modo_auto
    modo_auto = not modo_auto
    if modo_auto:
        modo_label.config(text="Modo: Automático", fg="green")
        enviar_comando("A")
    else:
        modo_label.config(text="Modo: Manual", fg="blue")
        enviar_comando("M")


def cambiar_modo_lidar():
    global modo_lidar_nube

    # Si vamos a entrar a modo NUBE, limpiamos antes
    if not modo_lidar_nube:
        nube_lidar.clear()

    modo_lidar_nube = not modo_lidar_nube
    texto = "LiDAR: Nube" if modo_lidar_nube else "LiDAR: Radar"
    lidar_mode_btn.config(text=texto)

def zoom_mas():
    global ESCALA_LIDAR
    ESCALA_LIDAR -= PASO_ZOOM
    if ESCALA_LIDAR < ESCALA_MIN:
        ESCALA_LIDAR = ESCALA_MIN

def zoom_menos():
    global ESCALA_LIDAR
    ESCALA_LIDAR += PASO_ZOOM
    if ESCALA_LIDAR > ESCALA_MAX:
        ESCALA_LIDAR = ESCALA_MAX


def tecla_presionada(event):
    if not modo_auto:
        mapa = {"Up": "F", "Down": "B", "Left": "L", "Right": "R"}
        if event.keysym in mapa:
            enviar_comando(mapa[event.keysym])

def tecla_suelta(event):
    if not modo_auto:
        enviar_comando("S")


def iniciar_registro():
    global tomando_datos, ultimo_registro, CSV_PATH, tiempo_inicio_registro

    tomando_datos = True
    registro_temporal.clear()
    trayectoria.clear()
    nube_lidar.clear()
    ultimo_registro = 0
    tiempo_inicio_registro = time.time()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    CSV_PATH = os.path.join(BASE_DIR, f"registro_{timestamp}.csv")

    status_label.config(text="Grabando datos...", fg="green")
    timer_label.config(text="⏱ 00:00", fg="green")

def detener_registro():
    global tomando_datos, tiempo_inicio_registro
    tomando_datos = False
    tiempo_inicio_registro = None

    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "hora","latitud","longitud","altitud",
            "modo","direccion_manual",
            "estado_obstaculo","distancia_obstaculo_cm"
        ])
        writer.writerows(registro_temporal)

    timer_label.config(text="")
    messagebox.showinfo("Registro", f"CSV guardado en:\n{CSV_PATH}")


def dibujar_lidar():
    lidar_canvas.update_idletasks()
    w, h = lidar_canvas.winfo_width(), lidar_canvas.winfo_height()
    if w < 50 or h < 50:
        return

    lidar_canvas.delete("all")
    cx, cy = w / 2, h / 2


    if not modo_lidar_nube:
        for r in [50, 100, 150, 200]:
            rr = r / ESCALA_LIDAR
            lidar_canvas.create_oval(cx-rr, cy-rr, cx+rr, cy+rr, outline="#0a3")

        lidar_canvas.create_line(cx, 0, cx, h, fill="#055")
        lidar_canvas.create_line(0, cy, w, cy, fill="#055")

        lidar_canvas.create_oval(cx-8, cy-8, cx+8, cy+8, fill="cyan")
        lidar_canvas.create_text(cx, cy+22, text="🚗", fill="white")

        if obstaculo_mas_cercano:
            x, y, dist = obstaculo_mas_cercano
            px = cx + x / ESCALA_LIDAR
            py = cy - y / ESCALA_LIDAR
            color = "red" if dist < UMBRAL_ALERTA else \
                    "orange" if dist < UMBRAL_ALERTA*2 else "lime"
            lidar_canvas.create_oval(px-8, py-8, px+8, py+8,
                                     fill=color, outline="")
            lidar_canvas.create_text(px, py-14,
                                     text=f"{int(dist)} cm", fill="white")


    else:
        for x, y in nube_lidar:
            px = cx + x / ESCALA_LIDAR
            py = cy - y / ESCALA_LIDAR
            if 0 <= px <= w and 0 <= py <= h:
                dist = math.hypot(x, y)
                color = "red" if dist < UMBRAL_ALERTA else \
                        "orange" if dist < UMBRAL_ALERTA*2 else "lime"
                lidar_canvas.create_oval(px-1, py-1, px+1, py+1,
                                          fill=color, outline="")


def dibujar_mapa():
    global marker_actual, path_actual
    if not ultimo_gps:
        return

    lat, lon = ultimo_gps["lat"], ultimo_gps["lon"]
    mapa_widget.set_position(lat, lon)

    if not tomando_datos:
        if path_actual:
            path_actual.delete()
            path_actual = None
        if marker_actual:
            marker_actual.set_position(lat, lon)
        else:
            marker_actual = mapa_widget.set_marker(lat, lon)
    else:
        if marker_actual:
            marker_actual.delete()
            marker_actual = None
        if len(trayectoria) > 1:
            if path_actual:
                path_actual.delete()
            path_actual = mapa_widget.set_path(trayectoria)


def actualizar():
    global ultimo_registro

    while not serial_queue.empty():
        procesar_linea(serial_queue.get())

    dibujar_lidar()
    dibujar_mapa()

    if tomando_datos and tiempo_inicio_registro:
        t = int(time.time() - tiempo_inicio_registro)
        timer_label.config(text=f"⏱ {t//60:02d}:{t%60:02d}")

    ahora = time.time()
    if tomando_datos and ahora - ultimo_registro >= INTERVALO_REGISTRO:
        ultimo_registro = ahora

        lat = ultimo_gps.get("lat") if ultimo_gps else ""
        lon = ultimo_gps.get("lon") if ultimo_gps else ""
        alt = ultimo_gps.get("alt") if ultimo_gps else ""

        modo = "AUTO" if modo_auto else "MANUAL"

        if modo_auto:
            direccion = direccion_auto
        else:
            direccion = DIRECCIONES_MANUAL.get(ultima_direccion, "detenido")


        if obstaculo_mas_cercano:
            dist = int(obstaculo_mas_cercano[2])
            estado = "CERCA" if dist < UMBRAL_ALERTA else "LEJOS"
        else:
            dist = ""
            estado = ""

        registro_temporal.append([
            datetime.now().strftime("%H:%M:%S"),
            lat, lon, alt,
            modo, direccion,
            estado, dist
        ])

    root.after(100, actualizar)

def salir():
    cerrar_conexion_serial()
    root.destroy()


root = tk.Tk()
root.title("VEHICULO LIDAR GPS")
root.attributes("-fullscreen", True)

root.bind("<Escape>", lambda e: root.attributes("-fullscreen", False))
root.bind("<KeyPress>", tecla_presionada)
root.bind("<KeyRelease>", tecla_suelta)

main = tk.Frame(root)
main.pack(fill="both", expand=True)

top = tk.Frame(main)
top.pack(fill="both", expand=True)

# MAPA
mapa_frame = ttk.LabelFrame(top, text="🗺 Mapa")
mapa_frame.pack(side="left", fill="both", expand=True)
mapa_widget = TkinterMapView(mapa_frame)
mapa_widget.pack(fill="both", expand=True)
mapa_widget.set_zoom(18)

# LIDAR
lidar_frame = ttk.LabelFrame(top, text="🔦 Radar LiDAR")
lidar_frame.pack(side="right", fill="both", expand=True)

lidar_toolbar = tk.Frame(lidar_frame)
lidar_toolbar.pack(anchor="e", padx=5, pady=5)

lidar_mode_btn = tk.Button(
    lidar_toolbar, text="LiDAR: Radar",
    command=cambiar_modo_lidar
)
lidar_mode_btn.pack(side="left", padx=2)

tk.Button(
    lidar_toolbar, text="+", width=3,
    command=zoom_mas
).pack(side="left", padx=2)

tk.Button(
    lidar_toolbar, text="-", width=3,
    command=zoom_menos
).pack(side="left", padx=2)

lidar_canvas = tk.Canvas(lidar_frame, bg="black")
lidar_canvas.pack(fill="both", expand=True)

# CONTROLES
control = ttk.LabelFrame(main, text="⚙ Control")
control.pack(pady=5)

modo_label = tk.Label(control, text="Modo: Manual", fg="blue",
                      font=("Arial",12,"bold"))
modo_label.pack(side="left", padx=10)

tk.Button(control, text="Cambiar Modo", bg="#2196F3",
          fg="white", command=cambiar_modo).pack(side="left", padx=3)

puerto = tk.Entry(control)
puerto.insert(0, "COM3")
puerto.pack(side="left")

tk.Button(control, text="Conectar", bg="green", fg="white",
          command=lambda: abrir_conexion_serial(puerto.get())).pack(side="left", padx=3)

tk.Button(control, text="Cerrar", bg="red", fg="white",
          command=cerrar_conexion_serial).pack(side="left", padx=3)

status_label = tk.Label(control, text="Desconectado", fg="red")
status_label.pack(side="left", padx=10)

timer_label = tk.Label(control, text="", font=("Arial",12,"bold"))
timer_label.pack(side="left", padx=10)

bottom = tk.Frame(main)
bottom.pack(pady=5)

tk.Button(bottom, text="Iniciar Registro", bg="green", fg="white",
          command=iniciar_registro).pack(side="left", padx=5)

tk.Button(bottom, text="Detener Registro", bg="orange", fg="black",
          command=detener_registro).pack(side="left", padx=5)

tk.Button(bottom, text="Salir", bg="red", fg="white",
          command=salir).pack(side="left", padx=5)

root.after(200, actualizar)
root.mainloop()


