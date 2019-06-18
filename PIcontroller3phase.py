import numpy as np

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
def store_Pcmd(storePcmd, Vang_meas):
    global Pcmd
    if Pcmd.size == 0:
        Pcmd = np.zeros((np.size(Vang_meas, 0), 1))
    if storePcmd == "read":
        return Pcmd
    else:
        Pcmd = storePcmd


# stores Qcmd
def store_Qcmd(storeQcmd, Vmag_meas):
    global Qcmd
    if Qcmd.size == 0:
        Qcmd = np.zeros((np.size(Vmag_meas, 0), 1))
    if storeQcmd == "read":
        return Qcmd
    else:
        Qcmd = storeQcmd


# P saturation counter
def P_sat(sat, Vang_meas):
    global Psat
    global Psat_counter
    global ICDI_sigP
    if Psat_counter.size == 0:
        Psat_counter = np.ones((np.size(Vang_meas), n))
        ICDI_sigP = np.zeros((np.size(Vang_meas), 1), dtype=bool)
    if sat == "read":
        return Psat
    else:
        Psat = sat
        np.delete(Psat_counter, 0, 1)
        np.append(Psat_counter, Psat, 1)
        for i in np.where(~Psat_counter.any(axis=1))[0]:
            ICDI_sigP[i] = True
# Q saturation counter
def Q_sat(sat, Vmag_meas):
    global Qsat
    global Qsat_counter
    global ICDI_sigQ
    if Qsat_counter.size == 0:
        Qsat_counter = np.ones((np.size(Vmag_meas), n))
        ICDI_sigQ = np.zeros((np.size(Vmag_meas), 1), dtype=bool)
    if sat == "read":
        return Qsat
    else:
        Qsat = sat
        np.delete(Qsat_counter, 0, 1)
        np.append(Qsat_counter, Qsat, 1)
        for i in np.where(~Qsat_counter.any(axis=1))[0]:
            ICDI_sigQ[i] = True


# MAIN PI CONTROL (call on this function every time step/iteration) Takes in 6 signals: Vang_meas, Vmag_meas from uPMUs;
# Pact, Qact from actuators; Vmag_ref, Vang_ref from S-PBC. INPUTS MUST BE CONVERTED NUMPY ARRAY FIRST!
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

    # Checking for P saturation (anti-windup control)
    indexP = np.where(abs(Pact) < abs(store_Pcmd("read", Vang_meas)))[0]
    sat_arrayP = np.ones((np.size(Vang_meas), 1))
    for i in indexP:
        sat_arrayP[i] = 0
    P_sat(sat_arrayP, Vang_meas)
    returnSigP = ICDI_sigP.nonzero()[0]
    Pmax = np.empty((np.size(Vang_meas), 1))
    for i in returnSigP:
        Pmax[i] = Pact[i]

    # Checking for Q saturation (anti-windup control)
    indexQ = np.where(abs(Qact) < abs(store_Qcmd("read", Vmag_meas)))[0]
    sat_arrayQ = np.ones((np.size(Vmag_meas), 1))
    for i in indexQ:
        sat_arrayQ[i] = 0
    Q_sat(sat_arrayQ, Vmag_meas)
    returnSigQ = ICDI_sigQ.nonzero()[0]
    Qmax = np.empty((np.size(Vmag_meas), 1))
    for i in returnSigQ:
        Qmax[i] = Qact[i]

    # PI control algorithm
    currentIntError_ang = (Ki_ang * phasor_error_ang) * (P_sat("read", 0))
    Pcmd = (Kp_ang * phasor_error_ang) + (IntError_ang(currentIntError_ang))

    currentIntError_mag = (Ki_mag * phasor_error_mag) * (Q_sat("read", 0))
    Qcmd = (Kp_mag * phasor_error_mag) + (IntError_mag(currentIntError_mag))

    # store new P/Q cmd to compare against P/Q actuators for anti-windup control above
    store_Pcmd(Pcmd,0)
    store_Qcmd(Qcmd,0)
    # returns 6 signals: ICDI_sigP, ICDI_sigQ to S-PBC; Pcmd, Qcmd to actuator; Pmax, Qmax to S-PBC
    return (ICDI_sigP, ICDI_sigQ, Pcmd, Qcmd, Pmax, Qmax)


# INITIALIZATION
intErrorOld_mag = 0
intErrorOld_ang = 0
Pcmd = np.array([])
Qcmd = np.array([])
n = 5  # saturation counter limit
Psat_counter = np.array([])
Qsat_counter = np.array([])
Pact = np.array([])  # Initialization
Qact = np.array([])  # Initialization
