# PI CONTROLLER (HIL)

# stores and calculates integrated errors for P-delta
def IntError_ang(currentIntError_ang):
    global intErrorOld_ang
    intErrorTotal_ang = currentIntError_ang + intErrorOld_ang
    intErrorOld_ang = intErrorTotal_ang
    return intErrorTotal_ang


# stores and calculates integrated errors for Q-mag
def IntError_mag(currentIntError_mag):
    global intErrorOld_mag
    intErrorTotal_mag = currentIntError_mag + intErrorOld_mag
    intErrorOld_mag = intErrorTotal_mag
    return intErrorTotal_mag


# stores Pcmd
def store_Pcmd(storePcmd):
    global Pcmd
    if storePcmd == "read":
        return Pcmd
    else:
        Pcmd = storePcmd


# stores Qcmd
def store_Qcmd(storeQcmd):
    global Qcmd
    if storeQcmd == "read":
        return Qcmd
    else:
        Qcmd = storeQcmd


# P saturation counter
def P_sat(sat):
    global Psat
    global ICDI_sigP
    if sat == "read":
        return Psat
    else:
        Psat = sat
        Psat_counter.pop(0)
        Psat_counter.append(Psat)
        if not any(Psat_counter):
            ICDI_sigP = True
        else:
            ICDI_sigP = False

# Q saturation counter
def Q_sat(sat):
    global Qsat
    global ICDI_sigQ
    if sat == "read":
        return Qsat
    else:
        Qsat = sat
        Qsat_counter.pop(0)
        Qsat_counter.append(Qsat)
        if not any(Qsat_counter):
            ICDI_sigQ = True
        else:
            ICDI_sigQ = False


# MAIN PI CONTROL (call on this function every time step/iteration) Takes in 6 signals: Vang_meas, Vmag_meas from uPMUs;
# Pact, Qact from actuators; Vmag_ref, Vang_ref from S-PBC.
def PIcontrol(Vang_meas, Vmag_meas, Pact, Qact, Vmag_targ, Vang_targ):
    global ICDI_sigP
    global ICDI_sigQ
    # Tuning parameters
    Kp_ang = 0.068
    Ki_ang = 0.037
    Kp_mag = 3.8
    Ki_mag = 2.15

    # Calculate phasor errors
    phasor_error_ang = Vang_targ - Vang_meas
    phasor_error_mag = Vmag_targ - Vmag_meas

    # Checking for saturation (anti-windup control)
    if Pact < store_Pcmd("read"):
        P_sat(0)
        if ICDI_sigP == True:
            Pmax = Pact
        else:
            Pmax = None
    else:
        P_sat(1)
        Pmax = None

    if Qact < store_Qcmd("read"):
        Q_sat(0)
        if ICDI_sigQ == True:
            Qmax = Qact
        else:
            Qmax = None
    else:
        Q_sat(1)
        Qmax = None

    # PI control algorithm
    currentIntError_ang = (Ki_ang * phasor_error_ang) * (P_sat("read"))
    Pcmd = (Kp_ang * phasor_error_ang) + (IntError_ang(currentIntError_ang))

    currentIntError_mag = (Ki_mag * phasor_error_mag) * (Q_sat("read"))
    Qcmd = (Kp_mag * phasor_error_mag) + (IntError_mag(currentIntError_mag))

    # store new P/Q cmd to compare against P/Q actuators for anti-windup control above
    store_Pcmd(Pcmd)
    store_Qcmd(Qcmd)
    # returns 6 signals: ICDI_sigP, ICDI_sigQ to S-PBC; Pcmd, Qcmd to actuator; Pmax, Qmax to S-PBC
    return (ICDI_sigP, ICDI_sigQ, Pcmd, Qcmd, Pmax, Qmax)


# INITIALIZATION
intErrorOld_mag = 0
intErrorOld_ang = 0
Pcmd = 0
Qcmd = 0
n = 5  # saturation counter limit
Psat_counter = [1] * n
Qsat_counter = [1] * n
Pact = 0  # Initialization
Qact = 0  # Initialization
Vang_ref = -120.721213337641  # hardcoded but should be from S-PBC. delete in HIL test
Vmag_ref = 1.00943273167702  # hardcoded but should be from S-PBC. delete in HIL test



