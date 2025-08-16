# -----------------------------------------
# CONFIGURAÇÃO E IMPORTAÇÕES
# -----------------------------------------
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, multitask, run_task


def zerar_heading_residual(alvo):
    """
    Define o heading atual como o erro residual em relação ao alvo.
    Isso permite que o próximo movimento (Gyro_Move, por exemplo)
    corrija o desvio deixado pelo giro anterior.
    """
    erro_residual = ((LaraZord.imu.heading() - alvo + 540) % 360) - 180
    LaraZord.imu.reset_heading(erro_residual)
    print(f"[✔] Heading residual ajustado para: {erro_residual:.4f}°")

# -----------------------------------------
# CONSTANTES E PARÂMETROS GLOBAIS
# -----------------------------------------
LaraZord = PrimeHub()
sensor_frente = ColorSensor(Port.E)
motoresquerdo = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motordireito = Motor(Port.B, Direction.CLOCKWISE)
drive_base = DriveBase(motoresquerdo, motordireito, 56, 110)

# PID padrão do seguidor
pid_p = 3.30
pid_i = 0.002
pid_d = 0.016

# Parâmetros do gyro_turn
GYRO_TOL = 0.5   # Tolerância para giro (graus)
GYRO_MIN = 50     # Potência mínima
GYRO_MAX = 80     # Potência máxima
GYRO_RESET = True

# -----------------------------------------
# VARIÁVEIS GLOBAIS MUTÁVEIS
# -----------------------------------------
erro = 0
correcao = 0
integral = 0
derivada = 0
erro_final = 0
guinadarefmove = 0
guinadacalc = 0
method_stop = 0

# -----------------------------------------
# FUNÇÕES AUXILIARES DE SENSOR/COR
# -----------------------------------------

def is_gray(hsv):
    h, s, v = hsv
    return 300 <= h <= 450 and 10 <= s <= 35 and 50 <= v <= 80

def is_amarelo(hsv):
    h, s, v = hsv
    return 40 <= h <= 80 and 30 <= s <= 60 and 80 <= v <= 120

def is_verde(hsv):
    h, s, v = hsv
    return 130 <= h <= 160 and 50 <= s <= 70 and 50 <= v <= 80

def is_azul(hsv):
    h, s, v = hsv
    return 200 <= h <= 250 and 50 <= s <= 60 and 50 <= v <= 70

async def cinza():
    hsv = await sensor_frente.hsv()
    print("HSV:", hsv)
    if is_gray(hsv):
        print("Carrinho CINZA detectado")
        await Gyro_Move(1.3, 60)
        drive_base.brake()
        await wait(200)
        await Gyro_Move(1.3, 70, True)
        drive_base.brake()
        await gyro_turn(90)
        drive_base.brake()
        await wait(20)
        await gyro_turn(90)
        drive_base.brake()
        await wait(20)
        await gyro_turn(90)
        drive_base.brake()

        return True
    return False

async def carrinhos():
    await wait(200)
    hsv = await sensor_frente.hsv()
    print("HSV:", hsv)
    if is_amarelo(hsv):
        print("Carrinho AMARELO detectado")
        await Gyro_Move(2.2, 60)
        drive_base.brake()
        await Gyro_Move(2.2, 70, True)
        return True
    elif is_verde(hsv):
        print("Carrinho VERDE detectado")
        await Gyro_Move(3.2, 60)
        drive_base.brake()
        await Gyro_Move(3.2, 70, True)
        return True
    elif is_azul(hsv):
        print("Carrinho AZUL detectado")
        await Gyro_Move(4.8, 60)
        drive_base.brake()
        await Gyro_Move(4.8, 70, True)
        return True
    else:
        print("Cor não detectada.")
        zerar_heading_residual(0)
        return False


# -----------------------------------------
# PID E UTILITÁRIOS
# -----------------------------------------
async def redefinir_pid():
    global erro_final, integral, derivada, correcao, erro
    erro_final = 0
    integral = 0
    derivada = 0
    correcao = 0
    erro = 0
    await wait(0)

async def PID(kp, ki, kd):
    global integral, derivada, correcao, erro_final
    integral += erro
    derivada = erro - erro_final
    correcao = erro * kp + (integral * ki + derivada * kd)
    erro_final = erro
    await wait(0)

async def redefinir():
    global erro_final, integral, derivada, correcao, erro, method_stop, pid_p, pid_i, pid_d, guinadacalc, guinadarefmove
    await wait(0)
    erro_final = 0
    integral = 0
    derivada = 0
    correcao = 0
    erro = 0
    method_stop = 0
    pid_p = 3.30
    pid_i = 0.002
    pid_d = 0.016
    guinadacalc = LaraZord.imu.tilt()[1]
    guinadarefmove = LaraZord.imu.tilt()[1]

# -----------------------------------------
# GIRO UNIVERSAL COM RESIDUAL
# -----------------------------------------
async def gyro_turn(graus):
    global erro, integral, derivada, correcao, erro_final
    global GYRO_TOL, GYRO_MIN, GYRO_MAX



    await redefinir_pid()

    kp = 5.7
    ki = 0.0006
    kd = 0.18



    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    while abs(erro_angular(alvo, LaraZord.imu.heading())) > GYRO_TOL:
        erro = erro_angular(alvo, LaraZord.imu.heading())
        await PID(kp, ki, kd)
        potencia = int(min(GYRO_MAX, max(GYRO_MIN, abs(correcao))))
        if erro > 0:
            motordireito.dc(potencia)
            motoresquerdo.dc(-potencia)
        else:
            motordireito.dc(-potencia)
            motoresquerdo.dc(potencia)
    motordireito.brake()
    motoresquerdo.brake()

    # Correção fina
    erro_residual = erro_angular(alvo, LaraZord.imu.heading())
    if abs(erro_residual) > 0.5:
        pulso_pot = 15
        pulso_tempo = 70
        if erro_residual > 0:
            motordireito.dc(-pulso_pot)
            motoresquerdo.dc(pulso_pot)
        else:
            motordireito.dc(pulso_pot)
            motoresquerdo.dc(-pulso_pot)
        await wait(pulso_tempo)
        motordireito.brake()
        motoresquerdo.brake()

    # --- Zeragem residual após o giro ---
    zerar_heading_residual(alvo)


# -----------------------------------------
# MOVIMENTO E GARRA
# -----------------------------------------
async def drive_straight(dist_mm, velocidade=200):
    drive_base.settings(straight_speed=velocidade)
    await drive_base.straight(dist_mm)
    drive_base.stop()

async def acionar_garra(power, ref):
    await garra.run_angle(power, ref)

# -----------------------------------------
# SEGUIDOR DE LINHA (PID CLÁSSICO) e outros movimentos
# -----------------------------------------
async def seguidor(velocidade):
    global erro
    while True:
        erro = await sensor_direito.reflection() - await sensor_esquerdo.reflection()
        await PID(pid_p, pid_i, pid_d)
        motoresquerdo.dc(velocidade - correcao)
        motordireito.dc(velocidade + correcao)
        await multitask(wait(0))


    # -----------------------------------------
# GYRO_MOVE COM AJUSTE RESIDUAL NO FINAL
# -----------------------------------------
async def Gyro_Move(rotacoes, velocidade_final, reverso=False):
    global erro, correcao
    await redefinir_pid()
    motoresquerdo.reset_angle(0)
    await wait(0)

    alvo_heading = 0

    # Configuração de aceleração
    velocidade_final = abs(velocidade_final)
    velocidade_atual = 30
    incremento = 2
    desacelera_a_partir = 0.8 * abs(rotacoes)

    sinal = -1 if reverso else 1
    corr_sinal = -1 if reverso else 1

    motoresquerdo.dc(sinal * (velocidade_atual + corr_sinal * correcao))
    motordireito.dc(sinal * (velocidade_atual - corr_sinal * correcao))

    while True:
        rot_atual = abs(motoresquerdo.angle() / 360)
        if rot_atual >= abs(rotacoes):
            break


        erro = ((alvo_heading - LaraZord.imu.heading() + 540) % 360) - 180

        if reverso:
            await PID(8.5, 0.002, 0.025)
        else:
            await PID(8.5, 0.002, 0.025)

        if rot_atual < 0.3 * abs(rotacoes):
            if velocidade_atual < velocidade_final:
                velocidade_atual += incremento
        elif rot_atual > desacelera_a_partir:
            velocidade_atual -= incremento
            if velocidade_atual < 50:
                velocidade_atual = 50

        if reverso:
            pot_esq = sinal * (velocidade_atual + correcao)
            pot_dir = sinal * (velocidade_atual - correcao)
        else:
            pot_esq = sinal * (velocidade_atual - correcao)
            pot_dir = sinal * (velocidade_atual + correcao)

        motoresquerdo.dc(pot_esq)
        motordireito.dc(pot_dir)


    motoresquerdo.brake()
    motordireito.brake()


    erro_residual = ((alvo_heading - LaraZord.imu.heading() + 540) % 360) - 180

    if abs(erro_residual) > 0.05:
        pulso_pot = 15
        pulso_tempo = 80
        print(f"Corrigindo heading final: erro = {erro_residual:.2f}°")

        if erro_residual > 0:
            motordireito.dc(-pulso_pot)
            motoresquerdo.dc(pulso_pot)
        else:
            motordireito.dc(pulso_pot)
            motoresquerdo.dc(-pulso_pot)

        await wait(pulso_tempo)
        motoresquerdo.brake()
        motordireito.brake()

    print("✅ Gyro_Move concluído. Heading atual:", LaraZord.imu.heading())
    zerar_heading_residual(alvo_heading)


async def main():
    await Gyro_Move(2.5, 50,False)
    await gyro_turn(90)
    await Gyro_Move(2.7, 80,False)
    await gyro_turn(90)
    await Gyro_Move(1.7, 80, False)
    await gyro_turn(-90)
    await Gyro_Move(1.7, 80,False)
    await wait (200)
    if await cinza():
        pass
    if await carrinhos():
        await gyro_turn(-90)
    await Gyro_Move (0.8, 80, False)
    await gyro_turn(90)
    await Gyro_Move (2.5, 80, False)
    await gyro_turn(-90)
    await Gyro_Move (1.3, 80, False)
    await wait (200)
    if await cinza():
        await Gyro_Move(0.5, 80, False)
        await gyro_turn(-90)
    if await carrinhos():
        await gyro_turn(-90)
        await Gyro_Move(0.5, 80, False)
        await gyro_turn(-90)
    await Gyro_Move(0.3, 80, False)
    await gyro_turn (90)
    zerar_heading_residual(0)
    await Gyro_Move(6.8, 80,False)
    await gyro_turn(90)
run_task(main())
