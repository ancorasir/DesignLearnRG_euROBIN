import rtde_control
import rtde_receive

import os
import sys
root_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.append(f"{root_dir}/lib")
import gripper

hande = gripper.Hand_E("/dev/ttyUSB0")
hande.connect()
hande.activate()

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")

poses = [
    [0.11088533771412556, -0.6056605109778147, 0.20404134740084689, -1.485134935039324, -2.744039039162598, 0.11622231906863742],
    [0.11088533771412556, -0.6056605109778147, 0.10404134740084689, -1.485134935039324, -2.744039039162598, 0.11622231906863742],
    [0.11088533771412556, -0.6056605109778147, 0.10404134740084689, -1.485134935039324, -2.744039039162598, 0.11622231906863742],
    [0.12538039701542178, -0.5980485347050671, 0.10290651870899356, -1.5634431825099617, -2.7046757929116345, 0.0933393860950454],
    [0.18091810727152266, -0.5417591580765633, 0.1047798359329402, -1.245476532538603, -2.8346650109804035, 0.5618018088893291],
    [0.12361459957619751, -0.5160744704688694, 0.07159726723417859, -0.3032719378445278, -3.0145170152515575, 0.7396108458724785],
    [-0.09670978288884169, -0.49062562405435794, 0.06401048032922627, -0.30072380753125674, -2.7974115323804045, 0.8611617717723072],
    [-0.034402178257458132, -0.5006321711223984, 0.07271181424887238, -0.3001791471281549, -2.9881156618014186, 0.904882019466892],
    
    [-0.039053986698064175, -0.5326989719126021, 0.11567562919601758, -0.3001791471281549, -2.9881156618014186, 0.904882019466892],
    # [-0.04127529446554556, -0.5829536119539325, 0.15790283437607736, -0.2701791471281549, -2.9881156618014186, 0.904882019466892],
    
    [-0.04106958401229917, -0.5464690779302083, 0.13017028005940393, -0.1395137331748882, -2.950939137060194, 0.9513919628423017],
    [0.1716126375701044, -0.570581120209573, 0.1224744109075978, -0.36202281754803852, -2.83454632835348, 1.1513439541043566],
    [0.1653764603193641, -0.5313745406907117, 0.05658252611874409, -0.30598556161744077, -2.9025684452245795, 0.9217532444402284],
    [-0.06889547430282997, -0.4984996850204337, 0.056411778644732236, -0.3080903918089969, -2.8645385507379837, 0.9054694037838089],
    [-0.03724918806228733, -0.5087012468405034, 0.056811569526083655, -0.3071521281683864, -2.939189862810455, 0.9359584265975678],
    
    [-0.038245811307478915, -0.5351291716006086, 0.10723563392079238, 0.3704680361447296, 2.9784751912339, -0.8716162994025098],

    [-0.0435317007778065, -0.5499501185402541, 0.13467437307169397, 0.40092782935529636, 2.954598021083465, -0.9351197003879997],
    [0.1580367120363804, -0.5640245446222419, 0.12533340204077386, -0.18462158610028717, -2.702915472209525, 1.2492971121468168],
    [0.1620146693396672, -0.5614163812102974, 0.08427310614257336, 0.8571516131188706, 2.7403520582893166, -1.2048559234641305],
    
    [0.12370781577601936, -0.5225915370914943, 0.08990350832085408, -0.1478937329413948, -2.0787381393581086, 0.8675668445341679],
    [0.12370781577601936, -0.5225915370914943, 0.15048012274289665, 0.292502945465351, 3.0895120823092497, -0.17269522152452937]
]

gripper_pos = [
    120,
    120,
    190,
    230,
    250,
    250,
    250,
    250,

    250,
    # 250,

    250,
    250,
    250,
    250,
    250,
    250,
    250,
    250,
    0,
    0,
    0
]

for i in range(len(poses)):
    input("Press anything to continue")
    print(i)
    print("Pose: ", poses[i])
    print("Gripper: ", gripper_pos[i])
    rtde_c.moveL(poses[i])
    hande.setPosition(gripper_pos[i])