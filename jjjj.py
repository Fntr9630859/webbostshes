from re import X

from matplotlib.contour import ContourSet

import os.path

useCV = False
try:
    from controller import Robot  # Conexión del controlador al objeto Robot
    import math
    import struct
    import time
    import cv2

    useCV = True
    import numpy as np

    camara_victima = True  # Comprobamos la funcionalidad de librerias como opencv y numpy
    viewHSV = True
except:
    print("Librerias no instaladas")


class victima:
    ## definicion de las propiedades de la clase y asignación de valores por omisión
    def __init__(self, ti, va, ca, px, pz):
        self.tipo = ti
        self.valor = va
        self.camara = ca
        self.posx = px
        self.posz = pz

    ## Definición de metodos de asignación para las propiedades de la clase
    def setTipo(self, ti):
        self.tipo = ti

    def setValor(self, va):
        self.valor = va

    def setCamara(self, ca):
        self.camara = ca

    def setPosx(self, px):
        self.posx = px

    def setPosz(self, pz):
        self.posz = pz

    ##definción del metodo que ejecuta la operación y guarda la operación y resultado a la memoria
    def getgetTipo(self):
        return self.tipo

    def getValor(self):
        return self.valor

    def getCamara(self):
        return self.camara

    def getPosx(self):
        return self.posx

    def getPosz(self):
        return self.posz


class imagen_ref:
    ## definicion de las propiedades de la clase y asignación de valores por omisión
    def __init__(self, im, ti, ca, co, no):
        self.imagen = im
        self.tipo = ti
        self.camara = ca
        self.color = co
        self.nomim = no

    ## Definición de metodos de asignación para las propiedades de la clase
    def setNombre(self, im):
        self.imagen = im

    def setTipo(self, ti):
        self.tipo = ti

    def setCamara(self, ca):
        self.camara = ca

    def setColor(self, co):
        self.color = co

    def setNomim(self, no):
        self.nomim = no

    ##definción del metodo que ejecuta la operación y guarda la operación y resultado a la memoria
    def getImagen(self):
        return self.imagen

    def getTipo(self):
        return self.tipo

    def getCamara(self):
        return self.camara

    def getColor(self):
        return self.color

    def getNomim(self):
        return self.nomim


# Crear un objeto de la clase robot
robot = Robot()
ref_c = []
ref_l = []
ref_r = []


def enc_tipo(ind):
    if ind <= 5:
        tip = "H"
    elif (ind <= 11):
        tip = "S"
    elif (ind <= 17):
        tip = "U"
    elif (ind <= 23):
        tip = "C"
    elif (ind <= 29):
        tip = "P"
    elif (ind <= 35):
        tip = "F"
    elif (ind <= 41):
        tip = "O"
    return tip


def enc_color(ind):
    #            H    S       U      C       P     F      O   #
    if ind in [0, 1, 6, 7, 12, 13, 18, 19, 24, 25, 30, 31, 36, 37]:
        col_fdo = "B"
    elif ind in [2, 3, 8, 9, 14, 15, 20, 21, 26, 27, 32, 33, 38, 39]:
        col_fdo = "N"
    elif ind in [4, 5, 10, 11, 16, 17, 22, 23, 28, 29, 34, 35, 40, 41]:
        col_fdo = "V"
    return col_fdo


# Imagen de referencia victimas camara central
for indice in range(0, 42, 1):
    if indice <= 9:
        archivo = '../imagenes/IMG_Blanco/C_0' + str(
            indice) + '.jpg'
    else:
        archivo = '../imagenes/IMG_Blanco/C_' + str(
            indice) + '.jpg'
    image = imagen_ref(cv2.imread(archivo), enc_tipo(indice), "C", enc_color(indice), archivo)
    ref_c.append(image)  # imagen de referencia
# Imagen de referencia victima camara izquierda
for indice in range(0, 42, 1):
    if indice <= 9:
        archivo = '../imagenes/IMG_Blanco/I_0' + str(
            indice) + '.jpg'
    else:
        archivo = '../imagenes/IMG_Blanco/I_' + str(
            indice) + '.jpg'
    image = imagen_ref(cv2.imread(archivo), enc_tipo(indice), "L", enc_color(indice), archivo)
    ref_l.append(image)  # imagen de referencia
# Imagen de referencia victima camara derecha
for indice in range(0, 42, 1):
    if indice <= 9:
        archivo = '../imagenes/IMG_Blanco/D_0' + str(
            indice) + '.jpg'
    else:
        archivo = '../imagenes/IMG_Blanco/D_' + str(
            indice) + '.jpg'
    image = imagen_ref(cv2.imread(archivo), enc_tipo(indice), "R", enc_color(indice), archivo)
    ref_r.append(image)  # imagen de referencia
# print(ref_r[7].getImagen())
# print(ref_r[7].getTipo())

indice = 0
# Definición de variables
timeStep = 16
# speed_max = 6.28
speed_max = 4.5
speed_med = 4.3
speed_ordynary = 4
speed_baj = 3.5
count = 1
step_c = 1
step_giro = 150
step_u = 140
ts = 120  # 55 #79
tg = 65  # 100
lg = 70  # 125
envio_m = False

d_fd = 0  ## distancia a la pared sensor frontal derecho
d_fi = 0  ## distancia a la pared sensor frontal izquierdo
d_dd = 0  ## distancia a la pared sensor derecho delantero
d_dc = 0  ## distancia a la pared sensor derecho central
d_id = 0  ## distancia a la pared sensor izquierdo delantero
d_ic = 0  ## distancia a la pared sensor izquierdo central
l_fd = 1  ## valor logico existe pared sensor frontal derecho
l_fi = 1  ## valor logico existe pared frontal derecho
l_dd = 1  ## valor logico existe pared sensor  derecho delantero
l_dc = 1  ## valor logico existe pared sensor  derecho central
l_id = 1  ## valor logico existe pared sensor izquierdo delantero
l_ic = 1  ## valor logico existe pared sensor izquierdo central
inercia = 'F'  ## recuerda en que sentido se mueve el robot (F) al frente (I) izaquierda (D) derecha (U) en U
inercia_ant = 'F'
inaux = ""
cont_ant = 0
per_lect_gps = 0
x = 0
y = 0
z = 0
camG = ""
enc_victima = ""
tipo_victima = "N"
dat_victima = victima("", 0, "", 0, 0)
contGVic = 0
vi = []  ##Vector de inercias

muros = 180  ##distancia a los muros para tomar la pared
frente = 45  ##distancia al frente para tomar la pared
lejos = 250  ## distancia en la que los sensores no pueden sentrarse
mas_lejos = 800  ##distancia maxima en la que los sonsores detectan algo
media = 60  ## distancia ideal de la pared izquierda
pantano = 1  ## multiplicador por pantano

cada = 10
cuantos = 0
sigue = 0

## definir los movimientos
paso = 0
count_der = 35
count_izq = 35
count_fte = 80
inicio = 1  ## pasos iniciales para que no se vaya al pozo

# Victimas visuales
v_visual = False
dif_paredes = 10
victimTimer = 0
contC = 0
contA = 0
contR = 0
contVic = 0

########################################

# la definicion de las ruedas del robot
rueda_rdf = robot.getDevice("wheel2 motor")
rueda_rif = robot.getDevice("wheel1 motor")
rueda_rdf.setPosition(float('inf'))
rueda_rif.setPosition(float('inf'))

# sensores de distancia
sensor_fd = robot.getDevice("ps0")
sensor_fd.enable(timeStep)
sensor_fi = robot.getDevice("ps7")
sensor_fi.enable(timeStep)
sensor_dd = robot.getDevice("ps1")
sensor_dd.enable(timeStep)
sensor_dc = robot.getDevice("ps2")
sensor_dc.enable(timeStep)
sensor_id = robot.getDevice("ps6")
sensor_id.enable(timeStep)
sensor_ic = robot.getDevice("ps5")
sensor_ic.enable(timeStep)

# Camaras y sensor de color
c_color = robot.getDevice("colour_sensor")
c_color.enable(timeStep)
camera = robot.getDevice("camera_centre")
camera.enable(timeStep)
camerar = robot.getDevice("camera_right")
camerar.enable(timeStep)
cameral = robot.getDevice("camera_left")
cameral.enable(timeStep)

## Emisor y recividor
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

# Declara GPS
gps = robot.getDevice("gps")
# gps = robot.getGPS("gps")
gps.enable(timeStep)
lect_gps = gps.getValues()


#######################################################################################
#                         busqueda de victimas visuales                               #
#######################################################################################

def delay(ms):
    initTime = robot.getTime()  # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (
                robot.getTime() - initTime) * 1000.0 > ms:  # If time elapsed (converted into ms) is greater than value passed in
            break


def checkVic():
    global ref_c, ref_l, ref_r, indice, cam_enc
    arch_c = '../imagenes/cen/cen' + str(indice) + '.jpg'
    arch_l = '../imagenes/izq/izq' + str(indice) + '.jpg'
    arch_r = '../imagenes/der/der' + str(indice) + '.jpg'
    # print(archivo)
    camera.saveImage(arch_c, 100)  # graba la imagen de la camara
    imag_c = cv2.imread(arch_c)  # imagen que ve el robot
    cameral.saveImage(arch_l, 100)  # graba la imagen de la camara
    imag_l = cv2.imread(arch_l)  # imagen que ve el robot
    camerar.saveImage(arch_r, 100)  # graba la imagen de la camara
    imag_r = cv2.imread(arch_r)  # imagen que ve el robot
    indice = indice + 1
    posX = int(gps.getValues()[0] * 100)
    posZ = int(gps.getValues()[2] * 100)
    max_im = 0
    tip_im = ""
    reg_im = ""
    cam_im = ""
    col_im = ""
    cam_enc = ""
    for i in range(0, 42, 1):
        # print(i)
        resultado = cv2.matchTemplate(imag_c, ref_c[i].getImagen(), cv2.TM_CCOEFF_NORMED)
        min, max, pos_min, pos_max = cv2.minMaxLoc(resultado)
        # print('minimo ', min,' Maximo ', max,'Pos minimo ',pos_min,'Pos maximo ',pos_max)
        cam_im = ref_c[i].getCamara()
        col_im = ref_c[i].getColor()
        if (max > max_im and cam_im == "C" and col_im != "B"):
            max_im = max
            tip_im = ref_c[i].getTipo()
            cam_enc = "C"

    for i in range(0, 42, 1):
        resultado = cv2.matchTemplate(imag_l, ref_l[i].getImagen(), cv2.TM_CCOEFF_NORMED)
        min, max, pos_min, pos_max = cv2.minMaxLoc(resultado)
        # print('minimo ', min,' Maximo ', max,'Pos minimo ',pos_min,'Pos maximo ',pos_max)
        cam_im = ref_l[i].getCamara()
        col_im = ref_l[i].getColor()
        if (max > max_im and cam_im == "L" and col_im != "B"):
            max_im = max
            tip_im = ref_l[i].getTipo()
            cam_enc = "L"

    for i in range(0, 42, 1):
        resultado = cv2.matchTemplate(imag_r, ref_r[i].getImagen(), cv2.TM_CCOEFF_NORMED)
        min, max, pos_min, pos_max = cv2.minMaxLoc(resultado)
        # print('minimo ', min,' Maximo ', max,'Pos minimo ',pos_min,'Pos maximo ',pos_max)
        cam_im = ref_r[i].getCamara()
        col_im = ref_r[i].getColor()
        if (max > max_im and cam_im == "R" and col_im != "B"):
            max_im = max
            tip_im = ref_r[i].getTipo()
            cam_enc = "R"

    print('|maximo |', max_im, '| tipo |', tip_im, '| Camara |', cam_enc, '| Color |', col_im, '| archivo |', archivo)

    if (max_im > dat_victima.getValor()):
        dat_victima.setCamara(cam_im)
        dat_victima.setTipo(tip_im)
        dat_victima.setValor(max_im)
        dat_victima.setPosx(posX)
        dat_victima.setPosz(posZ)
    return max_im


def report():
    global envio_m, inercia, contC, enc_victima
    print("se envio el mensaje: ", envio_m)
    if envio_m == False:
        paro()
        print("enc_victima ", enc_victima)
        delay(1300)
        # print("Termina espera...")
        # victimType = bytes('H', "utf-8")
        victimType = bytes(dat_victima.getgetTipo(), "utf-8")
        posX = dat_victima.getPosx()
        posZ = dat_victima.getPosz()
        message = struct.pack("i i c", posX, posZ, victimType)
        print(message)
        emitter.send(message)
        robot.step(timeStep)
        envio_m = True
        contC = 0
        enc_victima = ""
        tipo_victima = ""
        dat_victima.setCamara("")
        dat_victima.setPosx(0)
        dat_victima.setPosz(0)
        dat_victima.setTipo("")
        dat_victima.setValor(0)
    else:
        envio_m = False
        contVic = 0


def leer_camaras():
    global enc_victima
    global tipo_victima

    val_act = checkVic()
    val_ant = dat_victima.getValor()
    if (val_act < val_ant):
        if val_ant > 0.88:  ##0.69
            enc_victima = dat_victima.getgetTipo()
            tipo_victima = enc_victima
        else:
            enc_victima = ""
            tipo_victima = ""
            dat_victima.setCamara("")
            dat_victima.setPosx(0)
            dat_victima.setPosz(0)
            dat_victima.setTipo("")
            dat_victima.setValor(0)
    else:
        enc_victima = ""
        tipo_victima = ""

    # print("victima ", enc_victima)


def inclinacionesVictim(cam):
    global inercia, contGVic, camG
    if contVic == 0:
        camG = cam

    if (camG == "L"):
        if (camG == "L" and l_ic == 0 and l_fi == 0):
            print("Opcion 1_L")
            inercia = "VIG"
            contGVic = 10
        elif (camG == "L" and l_ic == 0 and l_fi == 1):
            print("Opcion 2_L")
            inercia = "VI"
            contGVic = 50
        elif (camG == "L" and (l_id == 1 or l_fi == 1)):
            print("Opcion 3_L")
            inercia = "F"
            contGVic = 60

    elif (camG == "R"):
        if (camG == "R" and l_dc == 0 and l_fd == 0):
            print("Opcion 1_R")
            inercia = "VDG"
            contGVic = 10
        elif (camG == "D" and l_dc == 0 and l_fd == 1):
            print("Opcion 2_R")
            inercia = "VD"
            contGVic = 50
        elif (camG == "R" and (l_dd == 1 or l_fd == 1)):
            print("Opcion 3_R")
            inercia = "F"
            contGVic = 60

    elif (camG == "C"):
        if (camG == "C" and l_fd == 1 and l_fi == 1):
            print("Opcion 1_C")
            inercia = "VC"
            contGVic = 40
    else:
        inercia = "F"
        contGVic = 70
    print("entre a la inclinacion", inercia)


def spin():
    rueda_rif.setVelocity(speed_max * 0.6)
    rueda_rdf.setVelocity(speed_max * -0.6)


###############################################################################################

# Procedimiento de lectura de sensores de distancia
def lee_distancia():
    global d_fd
    global d_fi
    global d_dd
    global d_dc
    global d_id
    global d_ic
    global l_fd
    global l_fi
    global l_dd
    global l_dc
    global l_id
    global l_ic

    d_fd = int(sensor_fd.getValue() * 1000)
    d_fi = int(sensor_fi.getValue() * 1000)
    d_dd = int(sensor_dd.getValue() * 1000)
    d_dc = int(sensor_dc.getValue() * 1000)
    d_id = int(sensor_id.getValue() * 1000)
    d_ic = int(sensor_ic.getValue() * 1000)

    if (d_ic > 500 and d_id < frente):
        # print("estoy pagado a la pared izquierda")
        d_ic = 0

    if (d_dc > 500 and d_dd < frente):
        # print("estoy pagado a la pared derecha")
        d_dc = 0

    '''if (d_fd>600 and d_fi>600 and (d_dd<frente or d_id<frente)):
        print("estoy pagado a la pared frontal a la derecha")
        d_fd=0
        d_fi=0'''

    if (d_fd > 600 and d_dd < frente):
        # print("estoy pagado a la pared frontal a la derecha")
        d_fd = 0

    if (d_fi > 600 and d_id < frente):
        # print("estoy pagado a la pared frontal a la izquierda")
        d_fi = 0

    ## logicos al frente
    if (d_fd > frente):
        l_fd = 1  ## no hay pared al frente
    else:
        l_fd = 0  ## hay pared al frente

    if (d_fi > frente):
        l_fi = 1  ## no hay pared al frente
    else:
        l_fi = 0  ## hay pared al frente

    ## logicos a las paredes
    if (d_dd > muros):
        l_dd = 1  ## no hay pared a la derecha
    else:
        l_dd = 0  ## hay pared a la derecha

    if (d_dc > muros):
        l_dc = 1  ## no hay pared a la derecha
    else:
        l_dc = 0  ## hay pared a la derecha

    if (d_id > muros):
        l_id = 1  ## no hay pared a la izquierda
    else:
        l_id = 0  ## hay pared a la izquierda

    if (d_ic > muros):
        l_ic = 1  ## no hay pared a la izquierda
    else:
        l_ic = 0  ## hay pared a la izquierda

    # print('distancia fd ', d_fd,'  fi ', d_fi,' derecha delantero ',d_dd,' derecha central ',d_dc," izquierda delantero ",d_id," izquierda central ",d_ic)
    # print('logica fd ', l_fd,'  fi ', l_fi,' derecha delantero ',l_dd,' derecha central ',l_dc," izquierda delantero ",l_id," izquierda central ",l_ic)


# Lectura del color del piso
def prueba_color():
    global ts
    global tg
    global lg
    global step_c
    global speed_ordynary
    global pantano
    color = c_color.getImage()
    # print(color)
    speed_ordynary = 4
    pantano = 1
    if color in [b'\x8e\xde\xf4\xff', b'\x8c\xdc\xf3\xff', b'\x8d\xdd\xf4\xff', b'\x81\xd1\xed\xff',
                 b'\x81\xd2\xed\xff', b'\x81\xd1\xec\xff', b'\x80\xd0\xec\xff', b'\x7f\xd0\xec\xff',
                 b'\x82\xd3\xee\xff', b'\x83\xd3\xee\xff', b'\x83\xd4\xee\xff', b'\x89\xd9\xf2\xff',
                 b'\x7f\xcf\xeb\xff', b'\xfe\xfe\xfe\xff', b'\x84\xd4\xef\xff', b'~\xce\xea\xff',
                 b'\x7f\xd0\xeb\xff', b'\x88\xd8\xf1\xff', b'}\xcd\xea\xff', b'\x80\xd0\xec\xff',
                 b'\x82\xd2\xed\xff']:
        # print("------------------------------------------------------PANTANO")
        pantano = 2
        speed_ordynary = 2
    elif color in [b';;;\xff', b';;@\xff', b'ooo\xff', b'---\xff', b'<<<\xff']:
        # print("------------------------------------------------------TRAMPA")
        step_c = 55
        sal_trampa()


def lee_gps():
    global lect_gps
    global x
    global y
    global z
    global inercia
    lect_gps = gps.getValues()
    xa = x
    za = z
    x = int(lect_gps[0] * 1000)
    y = int(lect_gps[1] * 1000)
    z = int(lect_gps[2] * 1000)
    # print("leyendo gps x-",x,"y-",y,"z-",z)
    if (xa == x and za == z and inercia == "F"):
        print("Estancado")
        inercia = "U"


def paro():
    print("-------- paro ------")
    rueda_rdf.setVelocity(0)
    rueda_rif.setVelocity(0)


# sigue de frente
def adelante():
    global inercia
    inercia = 'F'
    # print("adelante------")

    diferencia = d_id - d_ic
    # Si el robot esta centrado con ic
    if (diferencia >= 50 and diferencia <= 70):
        # y id es correcto seguimos de frente normal
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_ordynary)
        # print("sin ajuste")
    elif (diferencia < 50):
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_max)
        # print("ajuste a la derecha")
    elif (diferencia > 70):
        rueda_rdf.setVelocity(speed_max)
        rueda_rif.setVelocity(speed_ordynary)
        # print("ajuste a la izquierda")


def salir():
    global inercia
    global count
    # print("salir ------", count)
    rueda_rdf.setVelocity(speed_ordynary)
    rueda_rif.setVelocity(speed_ordynary)
    count = count + (1 / pantano)


def term_salir():
    global inercia
    global count
    inercia = 'TS'
    # print("Terminar de salir ------", count)
    rueda_rdf.setVelocity(speed_ordynary)
    rueda_rif.setVelocity(speed_ordynary)
    count = count + (1 / pantano)


def izquierda():
    global count
    global inercia
    inercia = 'I'
    rueda_rdf.setVelocity(speed_ordynary)
    rueda_rif.setVelocity(speed_ordynary * -1)
    count = count + (1 / pantano)


def derecha():
    global count
    global inercia
    inercia = 'D'
    rueda_rdf.setVelocity(speed_ordynary * -1)
    rueda_rif.setVelocity(speed_ordynary)
    count = count + (1 / pantano)


# vuelta a la derecha
def giro_u():
    global inercia
    global inercia_ant
    global count
    global sigue
    global termina
    global ts
    ts = 140
    inercia = "U"
    count = count + (1 / pantano)
    if (count <= tg):
        # print("Inicia el Giro ", count, "tg ", tg)
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_ordynary * -1)
    elif (d_fi >= d_fd - 100 and count <= lg):
        # print("Termina el giro ", count, "lg ", lg, "Diferencia ", d_fi-d_fd)
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_ordynary * -1)
    elif (count <= ts and inercia_ant != 'R'):
        # print("Salir ", count," ts ", ts)
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_ordynary)
    else:
        inercia_ant = 'U'
        inercia = "F"
        ts = 120
        adelante()


def sal_trampa():
    global step_c
    global inercia
    global ts
    global tg
    global lg
    global count
    # Retrocede
    rueda_rdf.setVelocity(speed_max * -1)
    rueda_rif.setVelocity(speed_max * -1)
    if (inercia != 'R'):
        count = 0
    inercia = "R"
    count = count + (1 / pantano)
    # print("Salir de trampa  ", count," Pasos ",step_c)
    if count > 15:
        count = 0
        inercia = "U"


def alinear_izquierda():
    # print("alinear a la izquierda")
    rueda_rdf.setVelocity(speed_ordynary)
    rueda_rif.setVelocity(speed_ordynary * -1)


def alinear_derecha():
    # print("alinear a la derecha")
    rueda_rdf.setVelocity(speed_ordynary * -1)
    rueda_rif.setVelocity(speed_ordynary)


def ajustar():
    ajustar = False
    if (d_fd != 0):
        multiplo = d_fi / d_fd
        if (multiplo < 0.8 and l_fi == 1 and l_fd == 1 and l_ic == 1 and l_id == 1 and l_dc == 1 and l_dd == 1):
            # paro()
            # print("Estoy pedrido me alineo a la izquierda",multiplo)
            alinear_izquierda()
            ajustar = True
        elif (multiplo > 1.2 and l_fi == 1 and l_fd == 1 and l_ic == 1 and l_id == 1 and l_dc == 1 and l_dd == 1):
            # paro()
            # print("Estoy perdido me alineo a la derecha",multiplo)
            alinear_izquierda()
            # alinear_derecha()
            ajustar = True
    return ajustar


def direccion():
    global inercia
    global inercia_ant
    global sigue
    global step_c
    global count
    ins = 2
    # print("--------------",inercia, "anterior ", inercia_ant)
    ## no hay paredes al frente
    if (inercia == 'F'):
        if (ajustar() == False):
            # no hay pared al frente y hay pared a la izquierda
            if (l_fd == 1 and l_fi == 1 and (l_id == 0 or l_ic == 0)):
                adelante()
            # Hay pared al frente y no hay pared a la izquierda(or o and)
            elif ((l_fd == 0 and l_fi == 0) and (l_id == 1 or l_ic == 1)):
                step_c = 35
                count = 0
                izquierda()
            # No hay pared al frente y no hay pared a la izquierda
            elif ((l_fd == 1 and l_fi == 1) and (l_id == 1 or l_ic == 1)):
                if (inercia_ant == 'U'):
                    step_c = 35
                    count = 0
                    izquierda()
                else:
                    step_c = 50
                    count = 0
                    inercia = 'SI'
                    salir()
            elif ((l_fd == 0 and l_fi == 0) and (l_dc == 1)):
                step_c = 35
                count = 0
                derecha()
            elif (l_fd == 0 and l_fi == 0 and l_id == 0 and l_ic == 0 and l_dd == 0 and l_dc == 0):
                step_c = 35
                count = 0
                giro_u()
            else:
                adelante()
    elif inercia == 'S':
        salir()
        # print("Salir", count)
        if (count > step_c or l_fd == 0 or l_fi == 0):
            inercia_ant = 'S'
            inercia = "F"
            adelante()
    elif inercia == 'I':
        # print("aqui")
        izquierda()
        # print("Izquierda", count)
        if (count > step_c):
            inercia_ant = 'I'
            step_c = 40
            count = 0
            term_salir()
    elif inercia == 'D':
        derecha()
        # print("Derecha", count)
        if (count > step_c):
            inercia_ant = 'D'
            step_c = 40
            count = 0
            term_salir()
    elif inercia == 'SI':
        inercia_ant = 'SI'
        if (count < step_c):
            salir()
        else:
            step_c = 35
            count = 0
            izquierda()
    elif inercia == "R":
        inercia_ant = 'R'
        sal_trampa()
    elif inercia == "DT":
        derecha()
        # print("Salir de derecha trampa", count)
        if (count > step_c):
            inercia_ant = 'DT'
            step_c = 40
            count = 0
            term_salir()
    elif inercia == "IT":
        izquierda()
        # print("Salir de izquierda trampa", count)
        if (count > step_c):
            inercia_ant = 'IT'
            step_c = 40
            count = 0
            term_salir()
    elif inercia == "TS":
        term_salir()
        # print("Terminar de salir", count)
        if (count > step_c or l_fd == 0 or l_fi == 0):
            inercia_ant = 'TS'
            step_c = 0
            count = 0
            adelante()
    elif inercia == 'U':
        print("U")
        giro_u()
    elif inercia == "VI":
        print("acercar a victima_L")
        rueda_rdf.setVelocity(speed_ordynary)
        rueda_rif.setVelocity(speed_baj)
    elif inercia == "VD":
        print("acercar a victima_R")
        rueda_rif.setVelocity(speed_ordynary)
        rueda_rdf.setVelocity(speed_baj)
    elif inercia == "VIG":
        rueda_rdf.setVelocity(speed_max)
        rueda_rif.setVelocity(speed_baj)
    elif inercia == "VDG":
        rueda_rif.setVelocity(speed_max)
        rueda_rdf.setVelocity(speed_baj)
    elif inercia == "VC":
        rueda_rif.setVelocity(speed_ordynary)
        rueda_rdf.setVelocity(speed_ordynary)


## Programa principal (main)
while robot.step(timeStep) != -1:
    # print("---------------------------------------------------------------------")
    # print("inicio: ",inicio," paso: ", paso," count ",count," pantano ", pantano," per_gps", per_lect_gps)
    if (per_lect_gps == 25):
        lee_gps()
        per_lect_gps = 0
    else:
        per_lect_gps = per_lect_gps + 1

    prueba_color()
    lee_distancia()
    direccion()

    # paro()

    print("EntVictima = ", enc_victima)
    if (enc_victima != ""):
        inclinacionesVictim(cam_enc)
        val_act = checkVic()
        if contVic >= contGVic:
            report()
            inercia = "F"
            print("inercia al salir de reporte ", inercia)
            contVic = 0
            contGVic = 0
            camG = ""
        else:
            print("enc_victima=", enc_victima, " cont ", contVic)
            contVic = contVic + (1 / pantano)
    else:
        print("Leo camaras...")
        leer_camaras()