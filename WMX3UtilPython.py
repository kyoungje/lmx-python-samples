# Import WMX3 API library
from WMX3ApiPython import *
from WMX3ApiPython import constants

# Import Python libraries and declare utility functions
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib import rc

import multiprocessing
from time import sleep

# %matplotlib inline
# %config InlineBackend.figure_format='retina'
sns.set(style='whitegrid', palette='muted', font_scale=1.2)
HAPPY_COLORS_PALETTE = ["#01BEFE", "#FFDD00", "#FF7D00", "#FF006D", "#ADFF02", "#8F00FF"]
sns.set_palette(sns.color_palette(HAPPY_COLORS_PALETTE))

title_font = {
    'fontsize': 16,
    'fontweight': 'bold'
}

# Constants
INFINITE = int(0xFFFFFFFF)

# wmxMemLogStatus = MemoryLogStatus()
manager = multiprocessing.Manager()
retrunLogUpdater = manager.dict()
curLogChannel = None
overflowFlag = 0
history = [np.zeros((0,)), np.zeros((0,))]
logUpdateProcess = False

def IntitializeWMX3Util(wmx3Lib, UpdateProcessEnabled=False):
    global wmxLog
    global logUpdateProcess

    wmxLog = Log(wmx3Lib)
    logUpdateProcess = UpdateProcessEnabled


# Plot utility functions
def plotPosMotion(df):
    plt.plot(df['Timestamp'].head(1000), df['DcDiffAvg'].head(1000))
    plt.xlabel('Time [cycle]')
    plt.ylabel('Position [p]')
    plt.title('Position Plot')

# Plot the velocity  plot
def plotVelMotion(df):
    plt.plot(df['Timestamp'].head(1000), df['DcDiffAvg'].head(1000))
    plt.xlabel('Time [cycle]')
    plt.ylabel('Velocity [p/s]')
    plt.title('velocity  Plot')

def OP_STATE(state):
    if state == OperationState.Idle:
        return "Idle"
    elif state == OperationState.Pos:
        return "Pos"
    elif state == OperationState.Jog:
        return "Jog"
    elif state == OperationState.Home:
        return "Home"
    elif state == OperationState.Sync:
        return "Sync"
    elif state == OperationState.GantryHome:
        return "GantryHome"
    elif state == OperationState.Stop:
        return "Stop"
    elif state == OperationState.Intpl:
        return "Intpl"
    elif state == OperationState.List:
        return "List"
    elif state == OperationState.ConstLinearVelocity:
        return "ConstLinearVelocity"
    elif state == OperationState.Trq:
        return "Trq"
    elif state == OperationState.DirectControl:
        return "DirectControl"
    elif state == OperationState.PVT:
        return "PVT"
    elif state == OperationState.ECAM:
        return "ECAM"
    elif state == OperationState.SyncCatchUp:
        return "SyncCatchUp"
    elif state == OperationState.DancerControl:
        return "DancerControl"
    else:
        return ""

def dumpStatus(cmAxis):
    print(" 1.PosCmd    :%-11.3f" % cmAxis.posCmd)             # 1. PosCmd
    print(" 2.ActPos    :%-11.3f" % cmAxis.actualPos)          # 2. ActualPos
    print(" 3.ActVel    :%-11.3f" % cmAxis.actualVelocity)     # 3. ActualVelocity
    print(" 4.ActTrq    :%-11.3f" % cmAxis.actualTorque)       # 4. ActualTorque 
    print(" 5.AmpAlm    :%-3d"    % cmAxis.ampAlarm)           # 5. AmpAlarm
    print(" 6.AmpAlmCode:0x%05x"  % cmAxis.ampAlarmCode)       # 6. AmpAlarmCode 
    print(" 7.SrvOn     :%-3d"    % cmAxis.servoOn)            # 7. ServoOn
    print(" 8.HomeDone  :%-4d"    % cmAxis.homeDone)           # 8. HomeDone 
    print(" 9.InPos     :%-3d"    % cmAxis.inPos)              # 9. InPos
    print("10.NegLS     :%-3d"    % cmAxis.negativeLS)         # 10.negativeLS
    print("11.PosLS     :%-3d"    % cmAxis.positiveLS)         # 11.positiveLS
    print("12.HomeSw    :%-4d"    % cmAxis.homeSwitch)         # 12.homeSwitch
    print("13.OpState   :%s"      % OP_STATE(cmAxis.opState))  # 13.OperationState

def LogUpdateTask (channel, dumpFlag=True):
    global history
    history = [np.zeros((0,)), np.zeros((0,))]
   
    while True:
        updateLogdata, overflowFlag = UpdateLogData(channel, 0)

        if overflowFlag:
            print(f'Overflow: {overflowFlag}');
        if not updateLogdata or len(updateLogdata) == 0:
            break
        if dumpFlag:
            print(f'Read log buffer size: {len(updateLogdata)}')
            
        np_feedbackPos = [data.feedbackPos for data in updateLogdata]
        np_feedbackVelocity = [data.feedbackVelocity for data in updateLogdata]
        
        history[0] = np.concatenate((history[0], np_feedbackPos), axis=0)
        history[1] = np.concatenate((history[1], np_feedbackVelocity), axis=0)

    if dumpFlag:
        print(f'Updated data size: {history[0].size}, {history[1].size}')
        print(history)

    retrunLogUpdater['history'] = history


def CheckErrorCode(func, errCode):
    if errCode != ErrorCode.PyNone:
        if errCode >= 0x00000 and errCode <= 0x10000:
           lastErrString =  f"function: {func}, ErrorType: WMX3Api, ErrorCode: {errCode}"
        elif errCode >= 0x11000 and errCode <= 0x11FFF:
            lastErrString = f"function: {func}, ErrorType: Log, ErrorCode: {errCode}"
        else:
            lastErrString = f"function: {func}, ErrorType: Undefined, ErrorCode: {errCode}"
        
        print(lastErrString)
        return False
    return True

def IsAvailableLogChannel(channel):    
    ret, memLogStatus = wmxLog.GetMemoryLogStatus(channel)    
    if ret != ErrorCode.PyNone:
        CheckErrorCode("GetMemoryLogStatus", ret)
        return False
    
    if memLogStatus.logState != LogState.Idle or memLogStatus.bufferOpened == True:
        print(f"memLogStatus.LogState: {memLogStatus.logState}, memLogStatus.bufferOpened: {memLogStatus.bufferOpened}")
        return False
    else:
        return True

def _CheckMemoryLogChannel():
    channel = -1
    for curchannel in range(constants.maxLogChannel - 1, 1, -1):
        print(f"curchannel: {curchannel}")
        if IsAvailableLogChannel(curchannel):
            ret = wmxLog.OpenMemoryLogBuffer(curchannel);
            if ret == ErrorCode.PyNone:
                channel = curchannel   
                break
            else:
                CheckErrorCode("OpenMemoryLogBuffer", ret)                

    return channel

def _CloseMemoryLog(channel):
    retryCount = 100

    for i in range (retryCount):
        ret, memLogStatus = wmxLog.GetMemoryLogStatus(channel)
        if ret != ErrorCode.PyNone:
            CheckErrorCode("GetMemoryLogStatus", ret)
            return False

        if memLogStatus.logState == LogState.Idle:
           break

        ret = wmxLog.StopMemoryLog(channel)
        if ret != ErrorCode.PyNone:
            CheckErrorCode("StopMemoryLog", ret)
            return False

        sleep(5)
            
    if ret != ErrorCode.PyNone:
        CheckErrorCode("_CloseMemoryLog", ret)
        return False

    return True

def UpdateLogData(channel, axis=0):
    updatedLogdata = None
    overflowFlag = False

    ret, memLogData = wmxLog.GetMemoryLogData(channel)
    if ret != ErrorCode.PyNone:
        CheckErrorCode("GetMemoryLogData", ret)
        return None, False

    overflowFlag = memLogData.overflowFlag > 0

    updatedLogdata = [None]*memLogData.count
    for index in range(memLogData.count):
        logdata = memLogData.GetLogData(index)                                       
        updatedLogdata[index] = logdata.GetLogAxisData(axis)
    
    return updatedLogdata, overflowFlag

def StartLog():
    global curLogChannel
    
    if curLogChannel is not None:
        StopLog(curLogChannel)
    
    maxAxes = 2
    channel = _CheckMemoryLogChannel()
    updaterProc = None
    
    if channel != -1:
        axisSel = AxisSelection()
        
        axisSel.axisCount = maxAxes;
        for idx in range(maxAxes):
            axisSel.SetAxis(idx, idx)

        evi = CoreMotionEventInput()
        evi.inputFunction  = CoreMotionEventInputType.OpState;
        inputFuncArg = CoreMotionEventInputFunctionArguments_OpState()
        inputFuncArg.opState = OperationState.Pos
        inputFuncArg.axis = 0
        inputFuncArg.invert = 0
        evi.opState = inputFuncArg
        
        # Set the output function to None
        evo = EventApiEventOutput()
        evo.outputFunction = EventApiEventOutputType.PyNone;
       
        memOption = MemoryLogOptions()
        memOption.triggerEventCount = 0

        # Set the event to trigger the memory log
        # ret, eventId = wmxEventCtrl.SetEvent(evi, evo)
        # if ret != ErrorCode.PyNone:
        #     CheckErrorCode("SetEvent", ret)            
        # else:
        #     memOption.triggerEventCount = 1
        #     memOption.SetTriggerEventID(0, eventId)
        #     wmxEventCtrl.EnableEvent(eventId, 1)
    
        ret = wmxLog.SetMemoryLog(channel, axisSel, memOption)
        if ret != ErrorCode.PyNone:
            CheckErrorCode("SetMemoryLog", ret)
            return -1
        
        ret = wmxLog.StartMemoryLog(channel)
        if ret != ErrorCode.PyNone:
            CheckErrorCode("StartMemoryLog", ret)
            return -1

        curLogChannel = channel
        if logUpdateProcess:
            updaterProc = multiprocessing.Process(target=LogUpdateTask, args=(channel,))
            updaterProc.start()

    return channel, updaterProc

def StopLog(channel):
    if _CloseMemoryLog(channel):
        
        ret = wmxLog.CloseMemoryLogBuffer(channel);
        if ret != ErrorCode.PyNone:
            CheckErrorCode("CloseMemoryLogBuffer", ret)

def PauseLog(channel, updaterProc):
    ret = wmxLog.StopMemoryLog(channel)
    if ret != ErrorCode.PyNone:
        CheckErrorCode("StopMemoryLog in PauseLog()", ret)
    
    sleep(0.1)
    
    ret, memLogStatus = wmxLog.GetMemoryLogStatus(channel)
    if ret != ErrorCode.PyNone:
        CheckErrorCode("GetMemoryLogStatus in PauseLog()", ret)
        return False
                
    print(f"Memory Logging is paused. (samplesToCollect: {memLogStatus.samplesToCollect}, samplesCollected: {memLogStatus.samplesCollected})") 

    if updaterProc:
        updaterProc.join()
        print(f'UpdaterProc: {retrunLogUpdater.values()}')
    else:
        global history
        history = [np.zeros((0,)), np.zeros((0,))]
   
        while True:
            updateLogdata, overflowFlag = UpdateLogData(channel, 0)

            if overflowFlag:
                print(f'Overflow: {overflowFlag}');
            if not updateLogdata or len(updateLogdata) == 0:
                break
                
            np_feedbackPos = [data.feedbackPos for data in updateLogdata]
            np_feedbackVelocity = [data.feedbackVelocity for data in updateLogdata]
            
            history[0] = np.concatenate((history[0], np_feedbackPos), axis=0)
            history[1] = np.concatenate((history[1], np_feedbackVelocity), axis=0)
   

def DrawLogPlot(plotTitle, dumpFlag=False):
    global history

    if dumpFlag:
        print(f'Updated data size: {history[0].size}, {history[1].size}')
        print(history)
        
    # Plot position and velocity feedbacks
    fig, axs = plt.subplots(
      nrows=1,
      ncols=2,
      sharey=False,
      sharex=False,
      figsize=(25, 15)
    )
    
    ax = axs[0]
    ax.plot(history[0], label='Position', color='limegreen')
    ax.set_xlabel('Cycle')
    ax.set_title(f'{plotTitle}: Feedback Position', fontdict=title_font, pad=20)
    ax.ticklabel_format(useOffset=False)
    ax.legend(['Position'])
    
    ax = axs[1]
    ax.plot(history[1], label='Velocity', color='violet')
    ax.set_xlabel('Cycle')
    ax.set_title(f'{plotTitle}: Feedback Velocity', fontdict=title_font, pad=20)
    ax.legend(['Velocity'])
    
    fig.tight_layout();